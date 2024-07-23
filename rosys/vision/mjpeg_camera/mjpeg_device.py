import asyncio
import logging
from asyncio import Task
from collections.abc import AsyncGenerator

import httpx

from ...rosys import on_startup
from ..image_processing import remove_exif
from .vendors import mac_to_url


class MjpegDevice:

    def __init__(self, mac: str, ip: str, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None) -> None:
        self.mac = mac
        self.ip = ip
        self.index = index
        self.capture_task: Task | None = None
        self._image_buffer: bytearray | None = None
        self.authentication = None if username is None or password is None else httpx.DigestAuth(username, password)
        self.log = logging.getLogger('rosys.mjpeg_device ' + self.mac)
        url = mac_to_url(mac, ip, index=index)
        if url is None:
            raise ValueError(f'could not determine URL for {mac}')
        self.url = url

        self.start_capture_task()

    def start_capture_task(self) -> None:
        def create_capture_task() -> None:
            loop = asyncio.get_event_loop()
            self.capture_task = loop.create_task(self.run_capture_task())
        on_startup(create_capture_task)

    def restart_capture(self) -> None:
        self.shutdown()
        self.start_capture_task()

    async def run_capture_task(self) -> None:
        self.log.info('Capturing images from %s', self.url)

        async def stream() -> AsyncGenerator[bytearray, None]:
            async with httpx.AsyncClient() as client:
                assert self.url is not None
                try:
                    async with client.stream('GET', self.url, auth=self.authentication) as response:  # type: ignore
                        if response.status_code != 200:
                            self.log.error('could not connect to %s (credentials: %s): %s %s',
                                           self.url, self.authentication, response.status_code, response.reason_phrase)
                            return

                        buffer_size = 16 * 1024 * 1024
                        buffer = bytearray(buffer_size)
                        buffer_view = memoryview(buffer)
                        buffer_end = 0
                        header = None

                        byte_search_pos = 0

                        try:
                            async for chunk in response.aiter_bytes():
                                chunk_len = len(chunk)

                                if buffer_end + chunk_len > buffer_size:
                                    self.log.warning('Buffer overflow, resetting buffer')
                                    buffer_end = 0
                                    header = None
                                    continue
                                buffer_view[buffer_end:buffer_end + chunk_len] = chunk
                                buffer_end += chunk_len

                                while True:
                                    if header is None:
                                        header_pos = buffer.find(b'\xff\xd8', byte_search_pos, buffer_end)
                                        if header_pos == -1:
                                            break
                                        byte_search_pos = header_pos + 2
                                        header = header_pos
                                    else:
                                        footer_pos = buffer.find(b'\xff\xd9', byte_search_pos, buffer_end)
                                        if footer_pos == -1:
                                            break

                                        image_end = footer_pos + 2
                                        image_data = buffer[header:image_end]
                                        yield image_data

                                        buffer_view[:buffer_end - image_end] = buffer_view[image_end:buffer_end]
                                        header = None
                                        buffer_end -= footer_pos + 2
                                        byte_search_pos = 0
                        except httpx.ReadTimeout:
                            self.log.warning('Connection to %s timed out', self.url)
                except Exception as e:
                    self.log.warning('Connection to %s failed. Was something disconnected?\n%s', self.url, e)
                    raise e

        async for image in stream():
            self._image_buffer = image
        self.log.warning('Capture task stopped')
        self.capture_task = None

    def capture(self) -> bytes | None:
        image = self._image_buffer
        self._image_buffer = None
        return remove_exif(image) if image is not None else None

    def shutdown(self) -> None:
        if self.capture_task is not None:
            self.capture_task.cancel()
            self.capture_task = None

    async def get_fps(self) -> int:
        return 0

    async def set_fps(self, fps: int) -> None:
        pass

    async def get_resolution(self) -> tuple[int, int]:
        return 0, 0

    async def set_resolution(self, width: int, height: int) -> None:
        pass

    async def get_mirrored(self) -> bool:
        return False

    async def set_mirrored(self, mirrored: bool) -> None:
        pass
