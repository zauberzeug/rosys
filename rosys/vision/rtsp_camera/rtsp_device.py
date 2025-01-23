import asyncio
import logging
import shlex
import signal
import subprocess
from asyncio.subprocess import Process
from collections.abc import AsyncGenerator, Awaitable, Callable
from io import BytesIO

from nicegui import background_tasks

from ... import rosys
from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


class RtspDevice:

    def __init__(self, mac: str, ip: str, *,
                 substream: int, fps: int, on_new_image_data: Callable[[bytes, float], Awaitable | None]) -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device.' + self._mac)

        self._fps = fps
        self._substream = substream
        self._on_new_image_data = on_new_image_data

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
            except asyncio.TimeoutError:
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
            except asyncio.TimeoutError:
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

        async def stream() -> AsyncGenerator[bytes, None]:
            url = self.url
            self.log.debug('[%s] Starting gstreamer pipeline for %s', self._mac, url)
            # to try: replace avdec_h264 with nvh264dec ! nvvidconv (!videoconvert)
            command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! jpegenc ! fdsink'
            self.log.debug('[%s] Running command: %s', self._mac, command)
            process = await asyncio.create_subprocess_exec(
                *shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                limit=1024*1024*1024,
            )
            assert process.stdout is not None
            assert process.stderr is not None
            self._capture_process = process

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
