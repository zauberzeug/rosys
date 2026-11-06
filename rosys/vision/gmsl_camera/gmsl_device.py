from __future__ import annotations

import asyncio
import logging
import shlex
import signal
import subprocess
from asyncio.subprocess import Process
from collections.abc import Awaitable, Callable

import numpy as np
from nicegui import background_tasks

from ... import rosys
from ...geometry import Rectangle
from ..capture_device import CaptureDevice, ImageDataHandler
from ..gstreamer import STDERR_TAIL_SIZE, GDPPacket, GDPPayloadType, parse_caps_dimensions, read_tail
from ..image_rotation import ImageRotation

FLIP_METHODS = {
    ImageRotation.NONE: 0,
    ImageRotation.LEFT: 1,
    ImageRotation.UPSIDE_DOWN: 2,
    ImageRotation.RIGHT: 3,
}


def build_argus_command(sensor_id: int, *,
                        auto_exposure: bool = True,
                        exposure: float = 0.01,
                        auto_gain: bool = True,
                        gain: float = 1.0,
                        fps: int = 30,
                        width: int = 1920,
                        height: int = 1200,
                        rotation: ImageRotation = ImageRotation.NONE,
                        crop: Rectangle | None = None) -> str:
    """Build the `gst-launch-1.0` command for an Argus pipeline that writes RGB frames as GDP packets to stdout.

    Pinning the gain also pins the ISP's digital gain: it is a third brightness lever that
    otherwise stays automatic and compensates for the pinned exposure and analog gain.

    Crop and rotation run in the hardware converter, so transformed frames cost no CPU time.
    """
    source_args = [f'sensor-id={sensor_id}']
    if not auto_exposure:
        exposure_ns = int(exposure * 1e9)
        source_args.append(f'exposuretimerange="{exposure_ns} {exposure_ns}"')
    if not auto_gain:
        source_args.append(f'gainrange="{gain} {gain}"')
        source_args.append('ispdigitalgainrange="1 1"')
    flip = '' if rotation == ImageRotation.NONE else f' flip-method={FLIP_METHODS[rotation]}'
    return (
        f'gst-launch-1.0 --quiet nvarguscamerasrc {" ".join(source_args)} ! '
        f'video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1 ! '
        f'{_crop_stage(crop, width, height, rotation)}'
        f'nvvidconv{flip} ! video/x-raw,format=BGRx ! '
        'videoconvert ! video/x-raw,format=RGB ! '
        'queue max-size-buffers=1 leaky=downstream ! gdppay ! fdsink sync=false'
    )


def _crop_stage(crop: Rectangle | None, width: int, height: int, rotation: ImageRotation) -> str:
    """Return a conversion step that crops the frame before it gets rotated, or nothing if there is no crop.

    The crop is widened to what the converter can do exactly; the rotating step after it sizes its output itself.
    """
    if crop is None:
        return ''
    x_span = _aligned_span(crop.x, crop.x + crop.width, width)
    y_span = _aligned_span(crop.y, crop.y + crop.height, height)
    if x_span is None or y_span is None:
        return ''
    (left, right), (top, bottom) = x_span, y_span
    if rotation in (ImageRotation.LEFT, ImageRotation.RIGHT):
        # the converter rejects turning a frame whose long side exceeds 16 times its short side
        left, right = _widened(left, right, width, -(-(bottom - top) // 16))
        top, bottom = _widened(top, bottom, height, -(-(right - left) // 16))
    # without the explicit size, nvvidconv scales the crop back up to the input size
    return (f'nvvidconv left={left} top={top} right={right} bottom={bottom} ! '
            f'video/x-raw(memory:NVMM),width={right - left},height={bottom - top} ! ')


def _aligned_span(start: float, end: float, limit: int) -> tuple[int, int] | None:
    """Widen ``[start, end)`` to what nvvidconv crops exactly within ``[0, limit)``, or ``None`` if it lies outside.

    nvvidconv only crops exactly on even coordinates and fails on sides shorter than 16 pixels,
    and GStreamer pads RGB rows whose width is not divisible by 4, which the frame reader would reject.
    """
    first = max(0, int(start) // 2 * 2)
    last = min(limit, int(end))
    if last <= first:
        return None
    return _widened(first, last, limit, 16)


def _widened(first: int, last: int, limit: int, min_length: int) -> tuple[int, int]:
    """Grow ``[first, last)`` to at least ``min_length`` and a length divisible by 4, shifted back into the frame."""
    length = min(-(-max(last - first, min_length) // 4) * 4, limit // 4 * 4)
    first = min(first, (limit - length) // 2 * 2)
    return first, first + length


class GmslDevice(CaptureDevice):
    """Captures frames from a GMSL2/FPD-Link camera attached to an NVIDIA Jetson.

    The Argus stack (`nvarguscamerasrc`) is used instead of the raw Video4Linux path so the
    hardware ISP processes the frames while exposure and gain can still be pinned, e.g. for
    long-exposure capture. Its properties are fixed at pipeline construction time, so a
    parameter change rebuilds the pipeline.
    """

    def __init__(self,
                 sensor_id: int,
                 *,
                 on_new_image_data: ImageDataHandler,
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 auto_exposure: bool = True,
                 exposure: float = 0.01,
                 auto_gain: bool = True,
                 gain: float = 1.0,
                 fps: int = 30,
                 width: int = 1920,
                 height: int = 1200,
                 rotation: ImageRotation = ImageRotation.NONE,
                 crop: Rectangle | None = None,
                 reconnect_interval: float = 3.0) -> None:
        super().__init__(name=f'gmsl-{sensor_id}',
                         log=logging.getLogger(f'rosys.vision.gmsl_camera.gmsl_device.{sensor_id}'),
                         on_connect=on_connect,
                         reconnect_interval=reconnect_interval)
        self.sensor_id = sensor_id
        self._on_new_image_data = on_new_image_data

        self.auto_exposure = auto_exposure
        self.exposure = exposure
        self.auto_gain = auto_gain
        self.gain = gain
        self.fps = fps
        self.width = width
        self.height = height
        self.rotation = rotation
        self.crop = crop

        self._capture_process: Process | None = None
        self._restart_task: asyncio.Task | None = None

        self._start_capture_task()

    def build_command(self) -> str:
        if not self.auto_exposure and self.exposure * self.fps > 1:
            self.log.warning('exposure %.3fs exceeds the frame period at %dfps; lower the fps for long exposures',
                             self.exposure, self.fps)
        return build_argus_command(self.sensor_id,
                                   auto_exposure=self.auto_exposure,
                                   exposure=self.exposure,
                                   auto_gain=self.auto_gain,
                                   gain=self.gain,
                                   fps=self.fps,
                                   width=self.width,
                                   height=self.height,
                                   rotation=self.rotation,
                                   crop=self.crop)

    def request_restart(self) -> None:
        """Schedule a single pipeline restart after the current batch of parameter changes."""
        if self._restart_task is not None and not self._restart_task.done():
            return
        self._restart_task = background_tasks.create(self._debounced_restart(), name=f'gmsl restart {self.sensor_id}')

    async def _debounced_restart(self) -> None:
        await asyncio.sleep(0)  # let a synchronous batch of setters finish before rebuilding the pipeline
        if not self.is_active:
            return
        await self.restart_gstreamer()

    async def restart_gstreamer(self) -> None:
        await self.shutdown()
        self._start_capture_task()

    async def shutdown(self) -> None:
        if self._restart_task is not None and self._restart_task is not asyncio.current_task():
            self._restart_task.cancel()  # a pending restart must not revive a device that is being torn down
        await super().shutdown()

    async def _tear_down_session(self) -> None:
        process = self._capture_process
        if process is None:
            return
        self.log.debug('[%s] terminating gstreamer process', self._name)
        try:
            process.terminate()
            await asyncio.wait_for(process.wait(), timeout=5)
        except ProcessLookupError:
            pass  # process already exited on its own
        except TimeoutError:
            self.log.warning('[%s] timeout while waiting for gstreamer process to terminate', self._name)
        else:
            if self._capture_process is process:
                self._capture_process = None

    async def _run_session(self) -> None:
        command = self.build_command()
        self.log.debug('[%s] running command: %s', self._name, command)
        try:
            process = await asyncio.create_subprocess_exec(
                *shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError as e:
            self.log.error('[%s] could not start gstreamer pipeline: %s', self._name, e)
            return
        assert process.stdout is not None
        assert process.stderr is not None
        self._capture_process = process
        stderr_tail = background_tasks.create(read_tail(process.stderr, STDERR_TAIL_SIZE),
                                              name=f'stderr {self._name}')

        try:
            width: int | None = None
            height: int | None = None
            while process.returncode is None:
                try:
                    packet = await GDPPacket.read(process.stdout)
                except asyncio.IncompleteReadError:
                    break
                if packet.payload_type == GDPPayloadType.CAPS:
                    width, height = parse_caps_dimensions(packet.payload.decode('utf-8', 'ignore'))
                elif packet.payload_type == GDPPayloadType.BUFFER:
                    assert width is not None and height is not None
                    if width * height * 3 != len(packet.payload):
                        self.log.warning('[%s] unexpected buffer size %d for %dx%d',
                                         self._name, len(packet.payload), width, height)
                        continue
                    image = np.frombuffer(packet.payload, dtype=np.uint8).reshape(height, width, 3)
                    result = self._on_new_image_data(image, rosys.time())
                    if isinstance(result, Awaitable):
                        await result
                    if not self.is_connected:
                        await self._enter_streaming()

            try:
                await asyncio.wait_for(process.wait(), timeout=5)
            except TimeoutError:
                self.log.warning('[%s] timeout while waiting for gstreamer process to terminate', self._name)
                return
            if process.returncode not in (0, -signal.SIGTERM):
                error_message = (await stderr_tail).decode(errors='replace').strip()
                self.log.warning('[%s] gstreamer pipeline exited with code %s.\nstderr: %s',
                                 self._name, process.returncode, error_message)
        finally:
            if process.returncode is None:
                self.log.debug('[%s] terminating leftover gstreamer process', self._name)
                process.terminate()
            if self._capture_process is process:  # a concurrent session owns its own process
                self._capture_process = None
