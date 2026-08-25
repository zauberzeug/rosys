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
from typing import ClassVar, Literal

import numpy as np
from nicegui import background_tasks

from ... import rosys
from ...vision.image import ImageArray
from ..openipc_zauberzeug_settings_interface import OpenIpcZauberzeugSettingsInterface
from .arkvision_rtsp_interface import ArkVisionRtspInterface
from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


class RtspDevice:
    UNAUTHORIZED_RECONNECT_INTERVAL: ClassVar[float] = 60.0
    '''How long to wait between attempts while the camera rejects our credentials.

    The rejection is only inferred from gstreamer's stderr, so a false positive must slow the retries
    down rather than stop them.
    '''

    def __init__(self, mac: str, ip: str | None = None, *,
                 substream: int, fps: int, on_new_image_data: Callable[[ImageArray, float], Awaitable | None],
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 avdec: Literal['h264', 'h265'] = 'h264',
                 reconnect_interval: float = 3.0) -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device.' + self._mac)

        self._fps = fps
        self._substream = substream
        self._on_new_image_data = on_new_image_data
        self._on_connect = on_connect
        self._avdec: Literal['h264', 'h265'] = self._clamp_avdec(avdec)

        self._capture_task: asyncio.Task | None = None
        self._capture_process: Process | None = None
        self._authorized: bool = True
        self._warned_about_missing_url: bool = False
        self._warned_about_missing_settings: bool = False
        self.reconnect_interval = reconnect_interval
        self._should_run: bool = True

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
    def is_connected(self) -> bool:
        """Whether the gstreamer stream is currently running."""
        return self._capture_process is not None and self._capture_process.returncode is None

    @property
    def is_active(self) -> bool:
        """Whether the self-healing capture loop is alive (streaming or waiting to reconnect)."""
        return self._capture_task is not None and not self._capture_task.done()

    @property
    def authorized(self) -> bool:
        """Whether the last attempt was not rejected; ``False`` while backing off after a rejected login."""
        return self._authorized

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
        self._authorized = True  # a rejection was about the previous address
        if self.is_connected:
            assert self._capture_process is not None
            self._capture_process.terminate()

    @property
    def url(self) -> str | None:
        """The stream URL, or ``None`` when the address is unknown or no URL scheme is known for the mac."""
        if self._ip is None:
            return None
        return mac_to_url(self._mac, self._ip, self._substream)

    async def shutdown(self) -> None:
        self._should_run = False
        process = self._capture_process
        if process is not None:
            self.log.debug('[%s] Terminating gstreamer process', self._mac)
            process.terminate()
            try:
                await asyncio.wait_for(process.wait(), timeout=5)
            except TimeoutError:
                self.log.warning('[%s] Timeout while waiting for gstreamer process to terminate', self._mac)
            else:
                self.log.debug('[%s] Successfully shut down process (code %s)',
                               self._mac, process.returncode if process.returncode is not None else 'None')
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

    def _start_capture_task(self) -> None:
        self.log.debug('[%s] Starting capture loop', self._mac)
        if self._capture_task is not None and not self._capture_task.done():
            self.log.warning('[%s] capture loop already running', self._mac)
            return
        self._should_run = True
        self._capture_task = background_tasks.create(self._run_capture_task(), name=f'capture {self._mac}')

    async def _run_capture_task(self) -> None:
        """Keep a single gstreamer session alive, reconnecting after `reconnect_interval` when it ends.

        Runs until `shutdown()` cancels the task.
        """
        try:
            while self._should_run:
                # every attempt starts fresh: an earlier rejection says nothing about this one
                self._authorized = True
                try:
                    await self._run_gstreamer()
                except Exception:
                    self.log.exception('[%s] capture session failed', self._mac)
                if not self._should_run:
                    break
                if not self._authorized:
                    self.log.info('[%s] credentials rejected; retrying in %.1f s',
                                  self._mac, self.UNAUTHORIZED_RECONNECT_INTERVAL)
                elif self._ip is None:
                    self.log.debug('[%s] no address known; retrying in %.1f s', self._mac, self.reconnect_interval)
                elif self.url is None:
                    self._warn_about_missing_url()
                else:
                    self.log.info('[%s] stream ended; reconnecting in %.1f s', self._mac, self.reconnect_interval)
                await self._wait_before_retry()
        finally:
            if self._capture_task is asyncio.current_task():
                self._capture_task = None

    def _warn_about_missing_url(self) -> None:
        """Warn once that no URL can be built for this camera.

        The URL scheme follows from the mac, so retrying cannot help; warning on every attempt would
        only fill the log.
        """
        if self._warned_about_missing_url:
            return
        self._warned_about_missing_url = True
        self.log.warning('[%s] no RTSP URL known for vendor %s; this camera cannot be reached',
                         self._mac, mac_to_vendor(self._mac))

    async def _wait_before_retry(self) -> None:
        """Wait `reconnect_interval` before the next session, extending the wait while our login stays rejected.

        Waiting in `reconnect_interval` chunks re-reads the verdict, so a new address ends a long wait
        early instead of holding on to a verdict that was about the previous address.
        """
        await rosys.sleep(self.reconnect_interval)
        waited = self.reconnect_interval
        while self._should_run and not self._authorized and waited < self.UNAUTHORIZED_RECONNECT_INTERVAL:
            await rosys.sleep(self.reconnect_interval)
            waited += self.reconnect_interval

    async def restart_gstreamer(self) -> None:
        await self.shutdown()
        self._start_capture_task()

    async def _invoke_on_connect(self) -> None:
        """Notify the owner that a capture session has been (re-)established, e.g. to reapply camera parameters."""
        if self._on_connect is None:
            return
        result = self._on_connect()
        if isinstance(result, Awaitable):
            await result

    async def _run_gstreamer(self) -> None:
        if self.is_connected:
            self.log.warning('[%s] capture process already running', self._mac)
            return
        url = self.url
        if url is None:
            return

        async def stream() -> AsyncGenerator[ImageArray, None]:
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
            await self._invoke_on_connect()

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

        try:
            async for image in stream():
                timestamp = rosys.time()
                result = self._on_new_image_data(image, timestamp)
                if isinstance(result, Awaitable):
                    await result
            self.log.info('[%s] stream ended', self._mac)
        finally:
            process = self._capture_process
            if process is not None and process.returncode is None:
                self.log.debug('[%s] terminating leftover gstreamer process', self._mac)
                process.terminate()
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
