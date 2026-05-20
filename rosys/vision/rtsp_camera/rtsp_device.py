from __future__ import annotations

import asyncio
import logging
import os
import re
import shlex
import signal
import struct
import subprocess
from asyncio.subprocess import Process
from collections.abc import AsyncGenerator, Awaitable, Callable
from dataclasses import dataclass
from enum import Enum
from typing import Literal

import numpy as np
from nicegui import background_tasks

from ... import rosys
from ...vision.image import ImageArray
from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor

# Camera-side pipeline delay from sensor frame latch to vendor pack_ts
# assignment at VENC output. The on-wire RTP timestamp encodes this
# downstream-of-sensor moment, so to recover sensor-side wall-clock we
# subtract this constant from the decoded RTP-based timestamp.

# Measured 2026-05-20 with calibrate_pipeline_delay.py script.
CAMERA_PIPELINE_DELAY_S: float = 0.0985
CAMERA_PIPELINE_DELAY_CALIBRATED_AT: tuple[int, int] = (1280, 960)


class RtspDevice:

    def __init__(self, mac: str, ip: str, *,
                 substream: int, fps: int, on_new_image_data: Callable[[ImageArray, float], Awaitable | None],
                 avdec: Literal['h264', 'h265'] = 'h264') -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device.' + self._mac)

        self._fps = fps
        self._substream = substream
        self._on_new_image_data = on_new_image_data
        self._avdec = avdec

        self._capture_task: asyncio.Task | None = None
        self._capture_process: Process | None = None
        self._authorized: bool = True

        vendor_type = mac_to_vendor(mac)

        self._settings_interface: JovisionInterface | None = None
        if vendor_type == VendorType.JOVISION:
            self._settings_interface = JovisionInterface(ip)
        else:
            self.log.warning('[%s] No settings interface for vendor type %s', self._mac, vendor_type)
            self.log.warning('[%s] Using default fps of 10', self._mac)

        self.log.info('[%s] Starting VideoStream for %s', self._mac, self.url)
        self._start_gstreamer_task()

    @property
    def is_connected(self) -> bool:
        return self._capture_task is not None

    @property
    def authorized(self) -> bool:
        return self._authorized

    @property
    def url(self) -> str:
        url = mac_to_url(self._mac, self._ip, self._substream)
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {self._mac}')
        return url

    async def shutdown(self) -> None:
        if self._capture_process is not None:
            self.log.debug('[%s] Terminating gstreamer process', self._mac)
            self._capture_process.terminate()
            try:
                await asyncio.wait_for(self._capture_process.wait(), timeout=5)
            except TimeoutError:
                self.log.warning('[%s] Timeout while waiting for gstreamer process to terminate', self._mac)
            else:
                self.log.debug('[%s] Successfully shut down process (code %s)',
                               self._mac, self._capture_process.returncode if self._capture_process.returncode is not None else 'None')
                self._capture_process = None
        if self._capture_task is not None and not self._capture_task.done():
            self.log.debug('[%s] Cancelling gstreamer task', self._mac)
            self._capture_task.cancel()
            try:
                await asyncio.wait_for(self._capture_task, timeout=5)
            except TimeoutError:
                self.log.warning('[%s] Timeout while waiting for capture task to cancel', self._mac)
                return
            except asyncio.CancelledError:
                self.log.debug('[%s] Task was successfully cancelled', self._mac)
            else:
                self.log.debug('[%s] Task finished', self._mac)
            self._capture_task = None

    def _start_gstreamer_task(self) -> None:
        self.log.debug('[%s] Starting gstreamer task', self._mac)
        if self._capture_task is not None and not self._capture_task.done():
            self.log.warning('[%s] capture task already running', self._mac)
            return
        self._capture_task = background_tasks.create(self._run_gstreamer(), name=f'capture {self._mac}')

    async def restart_gstreamer(self) -> None:
        await self.shutdown()
        self._start_gstreamer_task()

    async def _run_gstreamer(self) -> None:
        if self._capture_process is not None and self._capture_process.returncode is None:
            self.log.warning('[%s] capture process already running', self._mac)
            return

        async def stream() -> AsyncGenerator[tuple[ImageArray, float], None]:
            url = self.url
            self.log.debug('[%s] Starting gstreamer pipeline for %s', self._mac, url)

            # Second pipe in addition to stdout, see below.
            rtp_r_fd, rtp_w_fd = os.pipe()

            # tee splits the rtspsrc output: one branch decodes to RGB (the
            # existing path); the other forwards application/x-rtp buffers
            # so the per-frame on-wire RTP timestamp survives to user space.
            command = (
                f'gst-launch-1.0 --quiet '
                f'rtspsrc location="{url}" latency=20 buffer-mode=slave protocols=udp '
                f'! tee name=t '
                f't. ! queue max-size-buffers=200 max-size-bytes=10485760 max-size-time=1000000000 '
                f'! rtp{self._avdec}depay ! avdec_{self._avdec} ! videoconvert '
                f'! video/x-raw,format=RGB ! queue max-size-buffers=2 leaky=downstream '
                f'! gdppay ! fdsink fd=1 sync=false '
                f't. ! queue max-size-buffers=200 max-size-bytes=10485760 max-size-time=1000000000 '
                f'! gdppay ! fdsink fd={rtp_w_fd} sync=false'
            )
            self.log.debug('[%s] Running command: %s', self._mac, command)
            process = await asyncio.create_subprocess_exec(
                *shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                pass_fds=(rtp_w_fd,),
            )
            os.close(rtp_w_fd)
            assert process.stdout is not None
            assert process.stderr is not None
            self._capture_process = process

            loop = asyncio.get_running_loop()
            rtp_reader = asyncio.StreamReader(limit=2**20)
            rtp_pipe = os.fdopen(rtp_r_fd, 'rb', buffering=0)
            await loop.connect_read_pipe(
                lambda: asyncio.StreamReaderProtocol(rtp_reader),
                rtp_pipe,
            )

            pipeline_delay_warned = False

            # Maps gst running-time PTS (ns) -> on-wire 32-bit RTP timestamp.
            # The wrap window is chosen once so rtp_ts / 90 lands near host
            # wall-clock now; per-frame lookup then gives wall-clock directly,
            # independent of any gst PTS drift between tee branches.
            pts_to_rtp_ts: dict[int, int] = {}
            k_wrap: int | None = None
            wrap_ms = (1 << 32) / 90

            async def consume_rtp() -> None:
                nonlocal k_wrap
                while True:
                    try:
                        pkt = await GDPPacket.read(rtp_reader)
                    except asyncio.IncompleteReadError:
                        return
                    if pkt.payload_type != GDPPayloadType.BUFFER:
                        continue
                    if pkt.pts_ns == GST_CLOCK_TIME_NONE or len(pkt.payload) < 12:
                        continue
                    if pkt.pts_ns in pts_to_rtp_ts:
                        continue
                    rtp_ts = int.from_bytes(pkt.payload[4:8], 'big')
                    pts_to_rtp_ts[pkt.pts_ns] = rtp_ts
                    if k_wrap is None:
                        base_ms = rtp_ts / 90.0
                        k_wrap = round((rosys.time() * 1000 - base_ms) / wrap_ms)
                        self.log.info('[%s] RTP wrap anchor: rtp_ts=%d k=%d', self._mac, rtp_ts, k_wrap)
                    if len(pts_to_rtp_ts) > 1000:
                        for old in sorted(pts_to_rtp_ts)[:500]:
                            del pts_to_rtp_ts[old]

            rtp_task = asyncio.create_task(consume_rtp(), name=f'rtp-ts {self._mac}')

            try:
                width = None
                height = None
                while process.returncode is None:
                    try:
                        packet = await GDPPacket.read(process.stdout)
                    except asyncio.exceptions.IncompleteReadError:
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
                        assert width * height * 3 == len(packet.payload)
                        frame = np.frombuffer(packet.payload, dtype=np.uint8).reshape(height, width, 3)

                        rtp_ts = pts_to_rtp_ts.get(packet.pts_ns) if packet.pts_ns != GST_CLOCK_TIME_NONE else None
                        if rtp_ts is not None and k_wrap is not None:
                            capture_time = (rtp_ts / 90.0 + k_wrap * wrap_ms) / 1000
                            # The rtp_ts carries vendor-stamped time at VENC output,
                            # which is delayed from actual sensor latch by the
                            # camera-side pipeline (VPE + 3DNR + FRAMEBASE queue +
                            # encode). Subtract the measured delay to recover the
                            # moment the sensor captured the frame. The constant
                            # was calibrated for one specific resolution; if the
                            # stream resolution differs, the delay differs too, so
                            # we only apply the offset when the resolution matches.
                            if (width, height) == CAMERA_PIPELINE_DELAY_CALIBRATED_AT:
                                capture_time -= CAMERA_PIPELINE_DELAY_S
                            elif not pipeline_delay_warned:
                                pipeline_delay_warned = True
                                self.log.warning(
                                    '[%s] stream %dx%d does not match calibrated '
                                    '%dx%d; skipping pipeline-delay correction. '
                                    'Re-run utils/calibrate_pipeline_delay.py for this resolution.',
                                    self._mac, width, height,
                                    *CAMERA_PIPELINE_DELAY_CALIBRATED_AT)
                        else:
                            # RTP packet for this frame not yet seen on the second
                            # branch (typical for the first frame, or if the RTP
                            # branch is briefly behind); fall back to receive time.
                            self.log.info('[%s] using rosys time for frame', self._mac)
                            capture_time = rosys.time()
                        yield frame, capture_time

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

                    if 'Unauthorized' in error_message:
                        self._authorized = False
            finally:
                rtp_task.cancel()
                try:
                    await rtp_task
                except asyncio.CancelledError:
                    pass

        async for image, timestamp in stream():
            result = self._on_new_image_data(image, timestamp)
            if isinstance(result, Awaitable):
                await result

        self.log.info('[%s] stream ended', self._mac)

        self._capture_task = None

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
        self._avdec = avdec


class GDPPayloadType(Enum):
    NONE = 0
    BUFFER = 1
    CAPS = 2
    EVENT_NONE = 3


# GDP header layout (gst-plugins-bad 1.x):
# 0-1   version
# 2     flags
# 3     padding
# 4-5   payload type (1=BUFFER, 2=CAPS, large for events)
# 6-9   payload length
# 10-17 PTS (uint64 nanoseconds, GST_CLOCK_TIME_NONE if unset)
# 18-25 DTS or duration (we don't currently use either)
# 26-33 reserved/other (typically GST_CLOCK_TIME_NONE)
# 34-41 reserved/other (typically GST_CLOCK_TIME_NONE)
# 42-43 buffer flags
# 44-57 ABI padding
# 58-59 CRC header
# 60-61 CRC payload
GDPPACKET_FORMAT = struct.Struct('>HcxHIQQQQH14sHH')
GDP_CAPS_WIDTH_REGEX = re.compile(r'width=\(int\)\s*(\d+)')
GDP_CAPS_HEIGHT_REGEX = re.compile(r'height=\(int\)\s*(\d+)')
GDP_HEADER_SIZE = 62


GST_CLOCK_TIME_NONE = 0xFFFFFFFFFFFFFFFF


@dataclass(slots=True, kw_only=True)
class GDPPacket:
    payload_type: GDPPayloadType
    payload: bytes
    pts_ns: int  # presentation timestamp in nanoseconds, or GST_CLOCK_TIME_NONE if absent

    @staticmethod
    async def read(stream: asyncio.StreamReader) -> GDPPacket:
        header_bytes = await stream.readexactly(GDP_HEADER_SIZE)
        _version, _flags, gdp_type, length, pts, _q1, _q2, _q3, *_ = \
            GDPPACKET_FORMAT.unpack(header_bytes)
        return GDPPacket(
            payload_type=GDPPayloadType(gdp_type) if gdp_type < 3 else GDPPayloadType.EVENT_NONE,
            payload=await stream.readexactly(length),
            pts_ns=pts,
        )
