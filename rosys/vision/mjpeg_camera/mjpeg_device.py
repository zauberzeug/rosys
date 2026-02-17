import asyncio
import logging
from asyncio import Task
from collections.abc import AsyncGenerator, Awaitable, Callable

import httpx

from ... import rosys
from ...rosys import on_startup
from ..goodcam_interface import GoodCamInterface
from ..image_processing import remove_exif
from .motec_settings_interface import MotecSettingsInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


class _GoodCamMjpegSettings:
    """Adapts GoodCamInterface for MJPEG use with a fixed stream ID."""
    STREAM_ID = 1

    def __init__(self, interface: GoodCamInterface) -> None:
        self._interface = interface

    async def get_fps(self) -> int | None:
        return await self._interface.get_fps(self.STREAM_ID)

    async def set_fps(self, fps: int) -> None:
        await self._interface.set_fps(self.STREAM_ID, fps)

    async def get_resolution(self) -> tuple[int, int] | None:
        return await self._interface.get_resolution(self.STREAM_ID)

    async def set_resolution(self, width: int, height: int) -> None:
        await self._interface.set_resolution(self.STREAM_ID, width, height)

    async def get_bitrate(self) -> int | None:
        return None

    async def set_bitrate(self, bitrate: int) -> None:
        pass

    async def get_stream_compression(self) -> int:
        return await self._interface.get_quality(self.STREAM_ID) or 0

    async def set_stream_compression(self, level: int) -> None:
        await self._interface.set_quality(self.STREAM_ID, level)

    async def get_stream_port(self) -> int:
        return 0

    async def set_stream_port(self, port: int) -> None:
        pass


class _MotecMjpegSettings:
    """Adapts MotecSettingsInterface to the same interface as _GoodCamMjpegSettings."""

    def __init__(self, interface: MotecSettingsInterface) -> None:
        self._interface = interface

    async def get_fps(self) -> int | None:
        return await self._interface.get_fps()

    async def set_fps(self, fps: int) -> None:
        await self._interface.set_fps(fps)

    async def get_resolution(self) -> tuple[int, int] | None:
        return await self._interface.get_stream_resolution()

    async def set_resolution(self, width: int, height: int) -> None:
        await self._interface.set_stream_resolution(width, height)

    async def get_bitrate(self) -> int | None:
        return None

    async def set_bitrate(self, bitrate: int) -> None:
        pass

    async def get_stream_compression(self) -> int:
        return await self._interface.get_stream_compression()

    async def set_stream_compression(self, level: int) -> None:
        await self._interface.set_stream_compression(level)

    async def get_stream_port(self) -> int:
        return await self._interface.get_stream_port()

    async def set_stream_port(self, port: int) -> None:
        await self._interface.set_stream_port(port)


class MjpegDevice:

    def __init__(self, mac: str, ip: str, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 control_port: int | None = None,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None]) -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device.' + self._mac)

        self._on_new_image_data = on_new_image_data
        self._capture_task: Task | None = None
        self._authentication = None if username is None or password is None else httpx.DigestAuth(username, password)
        url = mac_to_url(mac, ip, index=index)
        if url is None:
            raise ValueError(f'could not determine URL for {mac}')
        self._url = url

        vendor_type = mac_to_vendor(mac)
        self._settings: _GoodCamMjpegSettings | _MotecMjpegSettings | None = None
        if vendor_type == VendorType.GOODCAM:
            goodcam = GoodCamInterface(ip, username=username or 'root', password=password or 'Adminadmin')
            self._settings = _GoodCamMjpegSettings(goodcam)
        elif vendor_type == VendorType.MOTEC:
            motec = MotecSettingsInterface(ip, port=control_port or 8885)
            self._settings = _MotecMjpegSettings(motec)

        self.start_capture_task()

    @property
    def is_connected(self) -> bool:
        return (self._capture_task is not None) and (not self._capture_task.done())

    def start_capture_task(self) -> None:
        def create_capture_task() -> None:
            loop = asyncio.get_event_loop()
            self._capture_task = loop.create_task(self.run_capture_task())
        on_startup(create_capture_task)

    def restart_capture(self) -> None:
        self.log.debug('Restarting capture task')
        self.shutdown()
        self.start_capture_task()

    async def run_capture_task(self) -> None:
        self.log.debug('Starting capture task for %s', self._url)

        async def stream() -> AsyncGenerator[bytes, None]:
            async with httpx.AsyncClient() as client:
                assert self._url is not None
                try:
                    async with client.stream('GET', self._url, auth=self._authentication) as response:  # type: ignore
                        if response.status_code != 200:
                            self.log.error('could not connect to %s (credentials: %s): %s %s',
                                           self._url, self._authentication, response.status_code, response.reason_phrase)
                            return

                        buffer_size = 16 * 1024 * 1024
                        buffer = bytearray(buffer_size)
                        buffer_view = memoryview(buffer)
                        buffer_end = 0

                        try:
                            async for chunk in response.aiter_bytes():
                                chunk_len = len(chunk)

                                if buffer_end + chunk_len > buffer_size:
                                    self.log.warning('Buffer overflow, resetting buffer')
                                    buffer_end = 0

                                buffer_view[buffer_end:buffer_end + chunk_len] = chunk
                                buffer_end += chunk_len

                                end = buffer.rfind(b'\xff\xd9', 0, buffer_end)
                                if end == -1:
                                    continue

                                start = buffer.rfind(b'\xff\xd8', 0, end)
                                if start == -1:
                                    continue

                                end += 2
                                yield buffer_view[start:end]
                                buffer_view[:buffer_end - end] = buffer_view[end:buffer_end]
                                buffer_end -= end

                            self.log.debug('Stream ended')
                        except httpx.ReadTimeout:
                            self.log.warning('Connection to %s timed out', self._url)
                except Exception as e:
                    self.log.warning('Connection to %s failed. Was something disconnected?\n%s', self._url, e)
                    raise e

        async for image in stream():
            if not image:
                continue
            try:
                timestamp = rosys.time()
                result = self._on_new_image_data(remove_exif(image), timestamp)
                if isinstance(result, Awaitable):
                    await result
            except Exception as e:
                self.log.error('Error processing image: %s', e)

        self.log.warning('Capture task stopped')
        self._capture_task = None

    def shutdown(self) -> None:
        self.log.debug('Shutting down capture task')
        if self._capture_task is not None:
            self._capture_task.cancel()
            self._capture_task = None

    async def get_fps(self) -> int:
        if self._settings is None:
            return 0
        return await self._settings.get_fps() or 0

    async def set_fps(self, fps: int) -> None:
        if self._settings is not None:
            await self._settings.set_fps(fps)

    async def get_resolution(self) -> tuple[int, int]:
        if self._settings is None:
            return (0, 0)
        return await self._settings.get_resolution() or (0, 0)

    async def set_resolution(self, width: int, height: int) -> None:
        if self._settings is not None:
            await self._settings.set_resolution(width, height)

    async def get_bitrate(self) -> int | None:
        if self._settings is None:
            return None
        return await self._settings.get_bitrate()

    async def set_bitrate(self, bitrate: int) -> None:
        if self._settings is not None:
            await self._settings.set_bitrate(bitrate)

    async def get_stream_compression(self) -> int:
        if self._settings is None:
            return 0
        return await self._settings.get_stream_compression()

    async def set_stream_compression(self, level: int) -> None:
        if self._settings is not None:
            await self._settings.set_stream_compression(level)

    async def get_stream_port(self) -> int:
        if self._settings is None:
            return 0
        return await self._settings.get_stream_port()

    async def set_stream_port(self, port: int) -> None:
        if self._settings is not None:
            await self._settings.set_stream_port(port)

    @property
    def settings_interface(self) -> _GoodCamMjpegSettings | _MotecMjpegSettings | None:
        return self._settings

    async def get_mirrored(self) -> bool:
        return False

    async def set_mirrored(self, mirrored: bool) -> None:
        pass
