from __future__ import annotations

import asyncio
import fcntl
import logging
import re
import shlex
import signal
import struct
import subprocess
from asyncio.subprocess import Process
from collections.abc import AsyncGenerator, Awaitable, Callable
from dataclasses import dataclass
from enum import Enum
from typing import Literal, cast

import cv2
import numpy as np

from ... import rosys
from ..capture_device import CaptureDevice, CaptureState, ImageDataHandler
from ..image import ImageArray
from ..openipc_zauberzeug_settings_interface import OpenIpcZauberzeugSettingsInterface
from .arkvision_rtsp_interface import ArkVisionRtspInterface
from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


class RtspDevice(CaptureDevice):

    def __init__(self, mac: str, ip: str | None = None, *,
                 substream: int, fps: int, on_new_image_data: ImageDataHandler,
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 avdec: Literal['h264', 'h265'] = 'h264',
                 reconnect_interval: float = 3.0) -> None:
        super().__init__(name=mac,
                         log=logging.getLogger('rosys.vision.rtsp_camera.rtsp_device.' + mac),
                         on_connect=on_connect,
                         reconnect_interval=reconnect_interval)
        self._mac = mac
        self._ip = ip

        self._fps = fps
        self._substream = substream
        self._on_new_image_data = on_new_image_data
        self._avdec: Literal['h264', 'h265'] = self._clamp_avdec(avdec)

        self._capture_process: Process | None = None
        self._warned_about_missing_url: bool = False
        self._warned_about_missing_settings: bool = False

        self._settings_interface: JovisionInterface | ArkVisionRtspInterface | OpenIpcZauberzeugSettingsInterface | None = None
        self._bind_settings_interface()

        self._start_capture_task()

    def _bind_settings_interface(self) -> None:
        """(Re-)create the vendor settings interface for the current address."""
        if self._ip is None:
            self._settings_interface = None
            return
        vendor_type = mac_to_vendor(self._mac)
        if vendor_type == VendorType.JOVISION:
            self._settings_interface = JovisionInterface(self._ip)
        elif vendor_type == VendorType.ARKVISION:
            self._settings_interface = ArkVisionRtspInterface(self._ip)
        elif vendor_type == VendorType.OPENIPC_ZAUBERZEUG:
            self._settings_interface = OpenIpcZauberzeugSettingsInterface(self._ip)
        else:
            self._settings_interface = None
            if not self._warned_about_missing_settings:  # rebinding on every address change must not spam the log
                self._warned_about_missing_settings = True
                self.log.warning('[%s] no settings interface for vendor type %s; keeping the configured fps',
                                 self._mac, vendor_type)

    @property
    def ip(self) -> str | None:
        """The address of the camera; assigning a new one makes the capture loop use it for its next session."""
        return self._ip

    @ip.setter
    def ip(self, ip: str | None) -> None:
        if ip == self._ip:
            return
        self.log.info('[%s] address changed to %s', self._mac, ip)
        self._ip = ip
        self._bind_settings_interface()
        self.restart_capture()

    @property
    def url(self) -> str | None:
        """The stream URL, or ``None`` when the address is unknown or no URL scheme is known for the mac."""
        if self._ip is None:
            return None
        return mac_to_url(self._mac, self._ip, self._substream)

    async def _tear_down_session(self) -> None:
        process = self._capture_process
        if process is None:
            return
        self.log.debug('[%s] Terminating gstreamer process', self._mac)
        process.terminate()
        try:
            await asyncio.wait_for(process.wait(), timeout=5)
        except TimeoutError:
            self.log.warning('[%s] Timeout while waiting for gstreamer process to terminate', self._mac)
        else:
            if self._capture_process is process:
                self._capture_process = None

    def _warn_about_missing_url(self) -> None:
        """Warn once that no URL can be built for this camera."""
        if self._warned_about_missing_url:
            return
        self._warned_about_missing_url = True
        self.log.warning('[%s] no RTSP URL known for vendor %s; this camera cannot be reached',
                         self._mac, mac_to_vendor(self._mac))

    def _retry_reason(self) -> tuple[int, str] | None:
        if self.is_refused:
            return logging.INFO, 'credentials rejected'
        if self._ip is None:
            return logging.DEBUG, 'no address known'
        if self.url is None:
            self._warn_about_missing_url()
            return logging.DEBUG, 'no stream URL known'
        return None

    async def restart_gstreamer(self) -> None:
        await self.shutdown()
        self._start_capture_task()

    async def _run_session(self) -> None:
        """Run a gstreamer session until it ends."""
        if self._capture_process is not None and self._capture_process.returncode is None:
            self.log.warning('[%s] capture process already running', self._mac)
            return
        url = self.url
        if url is None:
            return

        capture_process: Process | None = None

        async def stream() -> AsyncGenerator[ImageArray, None]:
            nonlocal capture_process
            self.log.debug('[%s] Starting gstreamer pipeline for %s', self._mac, url)
            hardware = await nvdec_is_available()
            # no parser between depay and nvv4l2decoder: an h265parse there negotiates a stream format the
            # hardware decoder accepts and then silently never emits a frame
            decoder = 'nvv4l2decoder ! nvvidconv' if hardware else f'avdec_{self._avdec} ! videoconvert'
            command = f'gst-launch-1.0 --quiet rtspsrc location="{url}" latency=0 protocols=tcp ! rtp{self._avdec}depay ! {decoder} ! video/x-raw,format=I420 ! queue max-size-buffers=1 leaky=downstream ! gdppay ! fdsink sync=false'
            self.log.debug('[%s] Running command: %s', self._mac, command)
            process = await asyncio.create_subprocess_exec(
                *shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                limit=STREAM_BUFFER_SIZE,
            )
            assert process.stdout is not None
            assert process.stderr is not None
            enlarge_pipe_buffer(process.stdout, self.log)
            self._capture_process = process
            capture_process = process

            width = None
            height = None
            delivered_a_frame = False
            while process.returncode is None:
                assert process.stdout is not None

                try:
                    read = GDPPacket.read(process.stdout)
                    # a hardware pipeline that cannot reach the decoder prerolls and then stalls silently,
                    # so the first frame is bounded to keep an unusable pipeline from blocking the camera
                    packet = await (asyncio.wait_for(read, timeout=NVDEC_FIRST_FRAME_TIMEOUT)
                                    if hardware and not delivered_a_frame else read)
                except asyncio.exceptions.IncompleteReadError:
                    break
                except TimeoutError:
                    self.log.warning('[%s] hardware decoding produced no frame within %.0f s; '
                                     'falling back to software decoding', self._mac, NVDEC_FIRST_FRAME_TIMEOUT)
                    disable_nvdec()
                    process.terminate()
                    break

                if packet.payload_type == GDPPayloadType.CAPS:
                    cap_text = packet.payload.decode('utf-8', 'ignore')

                    w = GDP_CAPS_WIDTH_REGEX.search(cap_text)
                    h = GDP_CAPS_HEIGHT_REGEX.search(cap_text)

                    assert w is not None and h is not None
                    assert len(w.groups()) == 1
                    assert len(h.groups()) == 1

                    width = int(w.group(1))
                    height = int(h.group(1))

                elif packet.payload_type == GDPPayloadType.BUFFER:
                    assert width is not None and height is not None

                    delivered_a_frame = True
                    yield i420_to_rgb(packet.payload, width, height)

            try:
                await asyncio.wait_for(process.wait(), timeout=5)
            except TimeoutError:
                self.log.warning(
                    '[%s] Stream ended. Timeout while waiting for gstreamer process to terminate', self._mac)
                return

            return_code = process.returncode
            if return_code == -1 * signal.SIGTERM:
                self.log.debug('gstreamer process %s was terminated using SIGTERM', process.pid)
            else:
                error = await process.stderr.read()
                error_message = error.decode()
                self.log.error('gstreamer process %s exited with code %s.\nstderr: %s',
                               process.pid, return_code, error_message)

                if 'Unauthorized' in error_message:  # inferred from stderr only, so back off rather than give up
                    self._set_state(CaptureState.REFUSED)

        try:
            async for image in stream():
                timestamp = rosys.time()
                result = self._on_new_image_data(image, timestamp)
                if isinstance(result, Awaitable):
                    await result
                if not self.is_connected:
                    await self._enter_streaming()
            self.log.info('[%s] stream ended', self._mac)
        finally:
            if capture_process is not None and capture_process.returncode is None:
                self.log.debug('[%s] terminating leftover gstreamer process', self._mac)
                capture_process.terminate()
            if self._capture_process is capture_process:  # a concurrent session owns its own process
                self._capture_process = None

    async def set_fps(self, fps: int) -> None:
        self._fps = fps

        if self._settings_interface is not None:
            await self._settings_interface.set_fps(stream_id=self._substream, fps=self._fps)

    async def get_fps(self) -> int | None:
        if self._settings_interface is not None:
            return await self._settings_interface.get_fps(stream_id=self._substream)
        return self._fps

    def set_substream(self, index: int) -> None:
        self._substream = index

    def get_substream(self) -> int:
        return self._substream

    async def set_bitrate(self, bitrate: int) -> None:
        if self._settings_interface is not None:
            await self._settings_interface.set_bitrate(stream_id=self._substream, bitrate=bitrate)

    async def get_bitrate(self) -> int | None:
        if self._settings_interface is not None:
            return await self._settings_interface.get_bitrate(stream_id=self._substream)
        return None

    def get_avdec(self) -> Literal['h264', 'h265'] | None:
        return self._avdec

    def set_avdec(self, avdec: Literal['h264', 'h265']) -> None:
        self._avdec = self._clamp_avdec(avdec)

    def _clamp_avdec(self, avdec: Literal['h264', 'h265']) -> Literal['h264', 'h265']:
        """ArkVision cameras only provide H.264, so force `avdec` to 'h264' for them."""
        if mac_to_vendor(self._mac) == VendorType.ARKVISION and avdec != 'h264':
            self.log.warning('[%s] ArkVision cameras only provide H.264; forcing avdec to "h264"', self._mac)
            return 'h264'
        return avdec


def enlarge_pipe_buffer(stream: asyncio.StreamReader, log: logging.Logger) -> None:
    """Grow the kernel pipe behind a stream, so a frame does not have to be handed over in 64 KiB pieces.

    The kernel caps this at ``/proc/sys/fs/pipe-max-size`` for unprivileged processes; falling short only
    costs throughput, so a rejected request is logged and ignored.
    """
    transport = getattr(stream, '_transport', None)
    pipe = transport.get_extra_info('pipe') if transport is not None else None
    if pipe is None:
        return
    try:
        fcntl.fcntl(pipe.fileno(), F_SETPIPE_SZ, PIPE_BUFFER_SIZE)
    except OSError as e:
        log.debug('could not grow the capture pipe to %d bytes: %s', PIPE_BUFFER_SIZE, e)


def i420_to_rgb(payload: bytes, width: int, height: int) -> ImageArray:
    """Convert an I420 buffer to RGB on the calling thread.

    Gstreamer pads each plane's rows to a four-byte stride, so a width that is not a multiple of four arrives
    wider than it is; the padding columns are dropped before converting.

    OpenCV would spread a conversion this small over its whole thread pool, where the dispatch costs several times
    the conversion itself.
    """
    luma_stride = (width + 3) & ~3
    chroma_stride = luma_stride // 2
    expected = luma_stride * height + 2 * chroma_stride * (height // 2)
    assert expected == len(payload), f'expected {expected} bytes for {width}x{height} I420, got {len(payload)}'

    buffer = np.frombuffer(payload, dtype=np.uint8)
    planes: np.ndarray
    if luma_stride == width:  # an unpadded buffer already has the layout cvtColor wants
        planes = buffer.reshape(height + height // 2, width)
    else:
        chroma_width = (width + 1) // 2
        chroma_height = height // 2
        planes = np.empty((height + chroma_height, width), dtype=np.uint8)
        planes[:height] = buffer[:luma_stride * height].reshape(height, luma_stride)[:, :width]
        chroma_rows = buffer[luma_stride * height:].reshape(2 * chroma_height, chroma_stride)[:, :chroma_width]
        planes[height:] = chroma_rows.reshape(chroma_height, 2 * chroma_width)

    threads = cv2.getNumThreads()
    cv2.setNumThreads(1)
    try:
        return cast(ImageArray, cv2.cvtColor(planes, cv2.COLOR_YUV2RGB_I420))
    finally:
        cv2.setNumThreads(threads)


class GDPPayloadType(Enum):
    NONE = 0
    BUFFER = 1
    CAPS = 2
    EVENT_NONE = 3


# See https://maemo.org/api_refs/5.0/5.0-final/gstreamer-libs-0.10/gstreamer-libs-gstdataprotocol.html for header format
GDPPACKET_FORMAT = struct.Struct('>HcxHIQQQQH14sHH')
GDP_CAPS_WIDTH_REGEX = re.compile(r'width=\(int\)\s*(\d+)')
GDP_CAPS_HEIGHT_REGEX = re.compile(r'height=\(int\)\s*(\d+)')
GDP_HEADER_SIZE = 62
STREAM_BUFFER_SIZE = 512 * 1024
"""Read buffer for the decoder pipe, large enough that a frame arrives without repeatedly pausing the transport."""
PIPE_BUFFER_SIZE = 256 * 1024
"""Kernel pipe capacity for the decoder pipe.

The default 64 KiB splits every frame into dozens of handovers, but the pipe is what buffers frames the
pipeline's leaky queue has already released: whatever fits here can still reach a stalled consumer as stale
images. A small-resolution substream has small frames, so this stays far below one frame of the largest
stream rather than being sized for the largest.
"""
F_SETPIPE_SZ = 1031


NVDEC_FIRST_FRAME_TIMEOUT = 15.0
"""How long a hardware pipeline may negotiate caps without delivering a frame before it counts as broken.

A decoder that cannot reach the hardware still prerolls and then stalls forever rather than failing, so a
timeout is the only signal that distinguishes it from a healthy but slow start.
"""

_nvdec_available: bool | None = None


async def nvdec_is_available() -> bool:
    """Whether to build a hardware-decoding pipeline.

    Presence of the element is necessary but not sufficient: it also loads where it cannot reach the
    hardware, so :func:`disable_nvdec` retires it when a pipeline proves unable to deliver frames.
    """
    global _nvdec_available  # noqa: PLW0603
    if _nvdec_available is None:
        try:
            process = await asyncio.create_subprocess_exec(
                'gst-inspect-1.0', 'nvv4l2decoder',
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            _nvdec_available = await asyncio.wait_for(process.wait(), timeout=10) == 0
        except (OSError, TimeoutError):
            _nvdec_available = False
    return _nvdec_available


def disable_nvdec() -> None:
    global _nvdec_available  # noqa: PLW0603
    _nvdec_available = False


@dataclass(slots=True, kw_only=True)
class GDPPacket:
    payload_type: GDPPayloadType
    payload: bytes

    @staticmethod
    async def read(stream: asyncio.StreamReader) -> GDPPacket:
        header_bytes = await stream.readexactly(GDP_HEADER_SIZE)
        _version, _flags, gdp_type, length, *_ = GDPPACKET_FORMAT.unpack(header_bytes)
        return GDPPacket(
            payload_type=GDPPayloadType(gdp_type) if gdp_type < 3 else GDPPayloadType.EVENT_NONE,
            payload=await stream.readexactly(length),
        )
