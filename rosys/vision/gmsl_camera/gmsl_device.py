from __future__ import annotations

import asyncio
import logging
import shlex
import signal
import subprocess
from asyncio.subprocess import Process
from collections.abc import AsyncGenerator, Awaitable, Callable

import numpy as np
from nicegui import background_tasks

from ... import rosys
from ..gstreamer import GDPPacket, GDPPayloadType, parse_caps_dimensions
from ..image import ImageArray


def build_argus_command(sensor_id: int, *,
                        auto_exposure: bool = True,
                        exposure: float = 0.01,
                        auto_gain: bool = True,
                        gain: float = 1.0,
                        fps: int = 30,
                        width: int = 1920,
                        height: int = 1200) -> str:
    """Build the `gst-launch-1.0` Argus pipeline command for the given configuration.

    The pipeline pulls frames from `nvarguscamerasrc` (hardware ISP: debayer/AWB/tonemap),
    converts to RGB and emits GDP packets on stdout via `gdppay ! fdsink`. Exposure and gain
    are left to the ISP's auto algorithms unless pinned to a fixed value.
    """
    source_args = [f'sensor-id={sensor_id}']
    if not auto_exposure:
        exposure_ns = int(exposure * 1e9)
        source_args.append(f'exposuretimerange="{exposure_ns} {exposure_ns}"')
    if not auto_gain:
        source_args.append(f'gainrange="{gain} {gain}"')
    return (
        f'gst-launch-1.0 --quiet nvarguscamerasrc {" ".join(source_args)} ! '
        f'video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1 ! '
        'nvvidconv ! video/x-raw,format=BGRx ! '
        'videoconvert ! video/x-raw,format=RGB ! '
        'queue max-size-buffers=1 leaky=downstream ! gdppay ! fdsink sync=false'
    )


class GmslDevice:
    """Captures frames from a GMSL2/FPD-Link camera attached to an NVIDIA Jetson.

    Frames are grabbed through NVIDIA's Argus stack (`nvarguscamerasrc`), so the
    hardware ISP performs debayering, auto white balance and tone mapping.
    Exposure and gain can be left to the ISP's auto algorithms or pinned to fixed
    values (e.g. for long-exposure capture), which is the main reason this backend
    is used instead of the raw Video4Linux path.

    The pipeline is run as a `gst-launch-1.0 ... ! gdppay ! fdsink` subprocess and
    its frames are read off a pipe, reusing the same mechanism as the RTSP camera.
    Because `nvarguscamerasrc` properties are fixed at pipeline construction time,
    changing a parameter restarts the pipeline (debounced, so a batch of changes
    only triggers a single restart).
    """

    def __init__(self,
                 sensor_id: int,
                 *,
                 on_new_image_data: Callable[[ImageArray, float], Awaitable | None],
                 auto_exposure: bool = True,
                 exposure: float = 0.01,
                 auto_gain: bool = True,
                 gain: float = 1.0,
                 fps: int = 30,
                 width: int = 1920,
                 height: int = 1200) -> None:
        self.sensor_id = sensor_id
        self.log = logging.getLogger(f'rosys.vision.gmsl_camera.gmsl_device.{sensor_id}')
        self._on_new_image_data = on_new_image_data

        self.auto_exposure = auto_exposure
        self.exposure = exposure
        self.auto_gain = auto_gain
        self.gain = gain
        self.fps = fps
        self.width = width
        self.height = height

        self._capture_task: asyncio.Task | None = None
        self._capture_process: Process | None = None
        self._restart_task: asyncio.Task | None = None

        self._start_gstreamer_task()

    @property
    def is_connected(self) -> bool:
        return self._capture_task is not None

    def build_command(self) -> str:
        """Build the `gst-launch-1.0` command for the current configuration."""
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
                                   height=self.height)

    def _start_gstreamer_task(self) -> None:
        if self._capture_task is not None and not self._capture_task.done():
            self.log.warning('capture task already running')
            return
        self._capture_task = background_tasks.create(self._run_gstreamer(), name=f'gmsl capture {self.sensor_id}')

    def request_restart(self) -> None:
        """Schedule a single pipeline restart after the current batch of parameter changes."""
        if self._restart_task is not None and not self._restart_task.done():
            return
        self._restart_task = background_tasks.create(self._debounced_restart(), name=f'gmsl restart {self.sensor_id}')

    async def _debounced_restart(self) -> None:
        await asyncio.sleep(0)  # let a synchronous batch of setters finish before rebuilding the pipeline
        await self.shutdown()
        self._start_gstreamer_task()

    async def shutdown(self) -> None:
        if self._capture_process is not None:
            self.log.debug('terminating gstreamer process')
            try:
                self._capture_process.terminate()
                await asyncio.wait_for(self._capture_process.wait(), timeout=5)
            except ProcessLookupError:
                pass  # process already exited on its own
            except TimeoutError:
                self.log.warning('timeout while waiting for gstreamer process to terminate')
            self._capture_process = None
        if self._capture_task is not None and not self._capture_task.done():
            self._capture_task.cancel()
            try:
                await asyncio.wait_for(self._capture_task, timeout=5)
            except TimeoutError:
                self.log.warning('timeout while waiting for capture task to cancel')
                return
            except asyncio.CancelledError:
                pass
        self._capture_task = None

    async def _run_gstreamer(self) -> None:
        if self._capture_process is not None and self._capture_process.returncode is None:
            self.log.warning('capture process already running')
            return

        async def stream() -> AsyncGenerator[ImageArray, None]:
            command = self.build_command()
            self.log.debug('running command: %s', command)
            try:
                process = await asyncio.create_subprocess_exec(
                    *shlex.split(command),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
            except OSError as e:
                self.log.error('could not start gstreamer pipeline: %s', e)
                return
            assert process.stdout is not None
            assert process.stderr is not None
            self._capture_process = process

            width: int | None = None
            height: int | None = None
            while process.returncode is None:
                try:
                    packet = await GDPPacket.read(process.stdout)
                except asyncio.IncompleteReadError:
                    break

                if packet.payload_type == GDPPayloadType.CAPS:
                    cap_text = packet.payload.decode('utf-8', 'ignore')
                    width, height = parse_caps_dimensions(cap_text)
                elif packet.payload_type == GDPPayloadType.BUFFER:
                    assert width is not None and height is not None
                    if width * height * 3 != len(packet.payload):
                        self.log.warning('unexpected buffer size %d for %dx%d', len(packet.payload), width, height)
                        continue
                    yield np.frombuffer(packet.payload, dtype=np.uint8).reshape(height, width, 3)

            try:
                await asyncio.wait_for(process.wait(), timeout=5)
            except TimeoutError:
                self.log.warning('stream ended; timeout while waiting for gstreamer process to terminate')
                return

            if process.returncode not in (0, -signal.SIGTERM):
                error_message = (await process.stderr.read()).decode()
                self.log.error('gstreamer process %s exited with code %s.\nstderr: %s',
                               process.pid, process.returncode, error_message)

        async for image in stream():
            result = self._on_new_image_data(image, rosys.time())
            if isinstance(result, Awaitable):
                await result

        self.log.info('stream ended')
        self._capture_process = None
        self._capture_task = None
