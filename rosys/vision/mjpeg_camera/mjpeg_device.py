import logging
from asyncio import Task
from asyncio.subprocess import Process
from io import BytesIO
from typing import AsyncGenerator, Optional

import httpx
from nicegui import background_tasks

from ..image_processing import remove_exif
from .vendors import mac_to_url


class MjpegDevice:

    def __init__(self, mac: str, ip: str, *,
                 index: Optional[int] = None, username: Optional[str] = None, password: Optional[str] = None) -> None:
        self.mac = mac
        self.ip = ip
        self.capture_task: Optional[Task] = None
        self.capture_process: Optional[Process] = None
        self._image_buffer: Optional[bytes] = None
        self._authorized: bool = True
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
        self.log.info(f'Capturing images from {self.url}')

        async def stream() -> AsyncGenerator[bytes, None]:
            async with httpx.AsyncClient() as client:
                assert self.url is not None
                try:
                    async with client.stream('GET', self.url, auth=self.authentication) as response:  # type: ignore
                        if response.status_code != 200:
                            self.log.error(f'could not connect to {self.url} '
                                           f'(credentials: {self.authentication}): '
                                           f'{response.status_code} {response.reason_phrase}')
                            return
                        buffer = BytesIO()
                        header = None
                        pos = 0
                        try:
                            async for chunk in response.aiter_bytes():
                                buffer.write(chunk)
                                while True:
                                    if header is None:
                                        header_pos = buffer.getvalue().find(b'\xff\xd8', pos)
                                        if header_pos == -1:
                                            pos = max(0, buffer.tell() - 1)
                                            break
                                        pos = header_pos + 2
                                        header = header_pos
                                    else:
                                        footer_pos = buffer.getvalue().find(b'\xff\xd9', pos)
                                        if footer_pos == -1:
                                            pos = max(0, buffer.tell() - 1)
                                            break
                                        image_data = buffer.getvalue()[header:footer_pos + 2]
                                        yield remove_exif(image_data)
                                        buffer = BytesIO(buffer.getvalue()[footer_pos + 2:])
                                        pos = 0
                                        header = None
                        except httpx.ReadTimeout:
                            self.log.warning(f'Connection to {self.url} timed out')
                except Exception:
                    self.log.warning(f'Initial connection to {self.url} failed. Was something disconnected?')

        async for image in stream():
            self._image_buffer = image

        self.capture_task = None

    def capture(self) -> Optional[bytes]:
        return self._image_buffer

    def shutdown(self) -> None:
        if self.capture_process is not None:
            self.capture_process.terminate()
            self.capture_process = None
