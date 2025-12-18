from __future__ import annotations

import asyncio
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
from typing import Literal

import numpy as np
from nicegui import background_tasks

from ... import rosys
from ...vision.image import ImageArray
from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


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

        async def stream() -> AsyncGenerator[ImageArray, None]:
            url = self.url
            self.log.debug('[%s] Starting gstreamer pipeline for %s', self._mac, url)
            # to try: replace avdec_h264 with nvh264dec ! nvvidconv (!videoconvert)
            command = f'gst-launch-1.0 --quiet rtspsrc location="{url}" latency=0 protocols=tcp ! rtp{self._avdec}depay ! avdec_{self._avdec} ! videoconvert ! video/x-raw,format=RGB ! queue max-size-buffers=1 leaky=downstream ! gdppay ! fdsink sync=false'
            self.log.debug('[%s] Running command: %s', self._mac, command)
            process = await asyncio.create_subprocess_exec(
                *shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            assert process.stdout is not None
            assert process.stderr is not None
            self._capture_process = process

            width = None
            height = None
            while process.returncode is None:
                assert process.stdout is not None

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

                    yield frame

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

        async for image in stream():
            timestamp = rosys.time()
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


# See https://maemo.org/api_refs/5.0/5.0-final/gstreamer-libs-0.10/gstreamer-libs-gstdataprotocol.html for header format
GDPPACKET_FORMAT = struct.Struct('>HcxHIQQQQH14sHH')
GDP_CAPS_WIDTH_REGEX = re.compile(r'width=\(int\)\s*(\d+)')
GDP_CAPS_HEIGHT_REGEX = re.compile(r'height=\(int\)\s*(\d+)')
GDP_HEADER_SIZE = 62


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
