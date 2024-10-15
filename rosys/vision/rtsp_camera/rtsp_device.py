import asyncio
import logging
import shlex
import signal
import subprocess
from asyncio.subprocess import Process
from collections.abc import AsyncGenerator, Awaitable, Callable
from io import BytesIO

from nicegui import background_tasks

from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


class RtspDevice:

    def __init__(self, mac: str, ip: str, *,
                 jovision_profile: int, fps: int, on_new_image_data: Callable[[bytes], Awaitable | None]) -> None:
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device')

        self.mac = mac
        self.ip = ip

        self.fps = fps
        self.jovision_profile = jovision_profile
        self.on_new_image_data = on_new_image_data

        self.capture_task: asyncio.Task | None = None
        self.capture_process: Process | None = None
        self._authorized: bool = True

        vendor_type = mac_to_vendor(mac)

        self.settings_interface: JovisionInterface | None = None
        if vendor_type == VendorType.JOVISION:
            self.settings_interface = JovisionInterface(ip)
        else:
            self.log.warning('[%s] No settings interface for vendor type %s', self.mac, vendor_type)
            self.log.warning('[%s] Using default fps of 10', self.mac)

        url = mac_to_url(mac, ip, jovision_profile)
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {mac}')
        self.log.info('[%s] Starting VideoStream for %s', self.mac, url)
        self._start_gstreamer_task()

    @property
    def authorized(self) -> bool:
        return self._authorized

    @property
    def url(self) -> str:
        url = mac_to_url(self.mac, self.ip, self.jovision_profile)
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {self.mac}')
        return url

    async def shutdown(self) -> None:
        if self.capture_process is not None:
            self.log.debug('[%s] Terminating gstreamer process', self.mac)
            self.capture_process.terminate()
            try:
                await asyncio.wait_for(self.capture_process.wait(), timeout=5)
            except asyncio.TimeoutError:
                self.log.warning('[%s] Timeout while waiting for gstreamer process to terminate', self.mac)
            else:
                self.log.debug('[%s] Successfully shut down process (code %s)',
                               self.mac, self.capture_process.returncode if self.capture_process.returncode is not None else 'None')
                self.capture_process = None
        if self.capture_task is not None and not self.capture_task.done():
            self.log.debug('[%s] Cancelling gstreamer task', self.mac)
            self.capture_task.cancel()
            try:
                await asyncio.wait_for(self.capture_task, timeout=5)
            except asyncio.TimeoutError:
                self.log.warning('[%s] Timeout while waiting for capture task to cancel', self.mac)
                return
            except asyncio.CancelledError:
                self.log.debug('[%s] Task was successfully cancelled', self.mac)
            else:
                self.log.debug('[%s] Task finished', self.mac)
            self.capture_task = None

    def _start_gstreamer_task(self) -> None:
        self.log.debug('[%s] Starting gstreamer task', self.mac)
        if self.capture_task is not None and not self.capture_task.done():
            self.log.warning('[%s] capture task already running', self.mac)
            return
        self.capture_task = background_tasks.create(self._run_gstreamer(), name=f'capture {self.mac}')

    async def restart_gstreamer(self) -> None:
        await self.shutdown()
        self._start_gstreamer_task()

    async def _run_gstreamer(self) -> None:
        if self.capture_process is not None and self.capture_process.returncode is None:
            self.log.warning('[%s] capture process already running', self.mac)
            return

        async def stream() -> AsyncGenerator[bytes, None]:
            url = self.url
            if 'subtype=0' in url:
                url = url.replace('subtype=0', 'subtype=1')

            self.log.debug('[%s] Starting gstreamer pipeline for %s', self.mac, url)
            # to try: replace avdec_h264 with nvh264dec ! nvvidconv (!videoconvert)
            command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! jpegenc ! fdsink'
            self.log.debug('[%s] Running command: %s', self.mac, command)
            process = await asyncio.create_subprocess_exec(
                *shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                limit=1024*1024*1024,
            )
            assert process.stdout is not None
            assert process.stderr is not None
            self.capture_process = process

            buffer = BytesIO()
            chunk_size = 1024*1024

            while process.returncode is None:
                assert process.stdout is not None

                eof = False
                while True:
                    new = await process.stdout.read(chunk_size)
                    eof = not new
                    buffer.write(new)
                    if len(new) < chunk_size:
                        break

                if eof:
                    break

                data = buffer.getvalue()
                end = data.rfind(b'\xff\xd9')
                if end == -1:
                    continue

                start = data.rfind(b'\xff\xd8', 0, end)
                if start == -1:
                    continue

                end += 2

                yield data[start:end]
                rest = data[end:]
                buffer.seek(0)
                buffer.truncate()
                buffer.write(rest)

            try:
                await asyncio.wait_for(process.wait(), timeout=5)
            except asyncio.TimeoutError:
                self.log.warning(
                    '[%s] Stream ended. Timeout while waiting for gstreamer process to terminate', self.mac)
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
            result = self.on_new_image_data(image)
            if isinstance(result, Awaitable):
                await result

        self.log.info('[%s] stream ended', self.mac)

        self.capture_task = None

    async def set_fps(self, fps: int) -> None:
        self.fps = fps

        if self.settings_interface is not None:
            await self.settings_interface.set_fps(stream_id=self.jovision_profile, fps=self.fps)

    async def get_fps(self) -> int | None:
        if self.settings_interface is not None:
            return await self.settings_interface.get_fps(stream_id=self.jovision_profile)
        return self.fps

    def set_jovision_profile(self, profile: int) -> None:
        self.jovision_profile = profile

    def get_jovision_profile(self) -> int:
        return self.jovision_profile

    async def set_bitrate(self, bitrate: int) -> None:
        if self.settings_interface is not None:
            await self.settings_interface.set_bitrate(stream_id=self.jovision_profile, bitrate=bitrate)

    async def get_bitrate(self) -> int | None:
        if self.settings_interface is not None:
            return await self.settings_interface.get_bitrate(stream_id=self.jovision_profile)
        return None
