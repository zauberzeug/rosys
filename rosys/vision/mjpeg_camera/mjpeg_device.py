import logging
# profiling
import time
from asyncio import Task
from typing import AsyncGenerator, Optional

import httpx
import numpy as np
from nicegui import background_tasks

import rosys

from ..image_processing import remove_exif
from .vendors import mac_to_url


class MjpegDevice:

    def __init__(self, mac: str, ip: str, *,
                 index: Optional[int] = None, username: Optional[str] = None, password: Optional[str] = None) -> None:
        self.mac = mac
        self.ip = ip
        self.capture_task: Optional[Task] = None
        self._image_buffer: Optional[bytearray] = None
        self.authentication = None if username is None or password is None else httpx.DigestAuth(username, password)
        self.log = logging.getLogger('rosys.mjpeg_device ' + self.mac)
        url = mac_to_url(mac, ip, index=index)
        if url is None:
            raise ValueError(f'could not determine URL for {mac}')
        self.url = url
        self.start_capture_task()

    def start_capture_task(self):
        self.capture_task = background_tasks.create(self.run_capture_task(), name=f'capture {self.mac}')

    async def restart_capture(self) -> None:
        self.shutdown()
        self.start_capture_task()

    async def run_capture_task(self) -> None:
        self.log.info('Capturing images from %s', self.url)

        async def stream() -> AsyncGenerator[bytearray, None]:
            async with httpx.AsyncClient() as client:
                assert self.url is not None
                try:
                    async with client.stream('GET', self.url, auth=self.authentication) as response:
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
                                        image_data = buffer[header:footer_pos + 2]
                                        yield image_data

                                        buffer_view[:buffer_end - (footer_pos + 2)
                                                    ] = buffer_view[footer_pos + 2:buffer_end]
                                        header = None
                                        buffer_end -= footer_pos + 2
                                        byte_search_pos = 0
                        except httpx.ReadTimeout:
                            self.log.warning('Connection to %s timed out', self.url)
                except Exception as e:
                    self.log.warning('Connection to %s failed. Was something disconnected?\n%s', self.url, e)
                    raise e
        i = 0
        async for image in stream():
            self._image_buffer = image
            i += 1

        self.capture_task = None

    def capture(self) -> Optional[bytes]:
        image = self._image_buffer
        self._image_buffer = None
        return remove_exif(bytes(image)) if image is not None else None

    def shutdown(self) -> None:
        if self.capture_task is not None:
            self.capture_task.cancel()
            self.capture_task = None
