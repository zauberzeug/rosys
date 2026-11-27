from __future__ import annotations

import asyncio
import logging
import shlex
import signal
import subprocess
from asyncio.subprocess import Process
from collections.abc import AsyncGenerator, Awaitable, Callable
from typing import Literal

import numpy as np
from nicegui import background_tasks

from ... import rosys
from ..capture_device import CaptureDevice, CaptureState, ImageDataHandler
from ..gstreamer import STDERR_TAIL_SIZE, GDPPacket, GDPPayloadType, parse_caps_dimensions, read_tail
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
            capture_process = process
            stderr_tail = background_tasks.create(read_tail(process.stderr, STDERR_TAIL_SIZE),
                                                  name=f'stderr {self._mac}')

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
                    width, height = parse_caps_dimensions(cap_text)

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
                error_message = (await stderr_tail).decode(errors='replace')
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
