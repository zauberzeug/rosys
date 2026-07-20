import asyncio
import logging
from asyncio import Task
from collections.abc import AsyncGenerator, AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager

import httpx

from ... import rosys
from ...geometry import Rectangle
from ...rosys import on_startup
from ..image import ImageArray
from ..image_processing import process_jpeg_image, remove_exif
from ..image_rotation import ImageRotation
from .vendors import mac_to_url

log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device')

ImageDataHandler = Callable[[ImageArray, float], Awaitable | None]
"""Receives a decoded, cropped and rotated frame together with its capture timestamp."""


def parse_capture_timestamp(part_header: bytes) -> float | None:
    """Extract the absolute capture instant (Unix epoch seconds) from the ``X-Timestamp``
    field of an MJPEG multipart part header.

    Cameras that stamp each frame at capture time emit an ``X-Timestamp: <sec>.<usec>``
    line in the part header preceding the JPEG. Returns ``None`` when the field is absent
    or unparsable, so the caller can fall back to the receive time.
    """
    marker = b'x-timestamp:'
    index = part_header.lower().rfind(marker)
    if index == -1:
        return None
    line_end = part_header.find(b'\r\n', index)
    raw = part_header[index + len(marker):] if line_end == -1 else part_header[index + len(marker):line_end]
    try:
        return float(raw.strip())
    except ValueError:
        return None


class MjpegDevice:

    def __init__(self, mac: str, ip: str, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: ImageDataHandler) -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device.' + self._mac)

        self._on_new_image_data = on_new_image_data
        self.rotation: ImageRotation = ImageRotation.NONE
        self.crop: Rectangle | None = None
        self._capture_task: Task | None = None
        self._username = username
        self._password = password
        url = mac_to_url(mac, ip, index=index)
        if url is None:
            raise ValueError(f'could not determine URL for {mac}')
        self._url = url

        self.start_capture_task()

    @property
    def is_connected(self) -> bool:
        return (self._capture_task is not None) and (not self._capture_task.done())

    def start_capture_task(self) -> None:
        def create_capture_task() -> None:
            loop = asyncio.get_event_loop()
            self._capture_task = loop.create_task(self.run_capture_task())
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            on_startup(create_capture_task)  # no loop yet (e.g. constructed at import time): defer to startup
        else:
            create_capture_task()  # a loop is already running (e.g. inside the capture subprocess)

    def restart_capture(self) -> None:
        self.log.debug('Restarting capture task')
        self.shutdown()
        self.start_capture_task()

    async def _prepare_stream(self) -> None:
        """Hook executed right before the MJPEG stream is opened (and on every restart).

        Vendors whose HTTP stream must be enabled before it serves data can override this.
        Implementations should log and return on failure rather than raise.
        """

    async def _frame_reader(self, response: httpx.Response) -> AsyncGenerator[tuple[bytes, float | None], None]:
        """Yield ``(jpeg, capture_time)`` pairs from a live MJPEG response."""
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

                # the bytes before the SOI marker are this frame's multipart part header
                capture_time = parse_capture_timestamp(bytes(buffer_view[:start]))
                end += 2
                yield bytes(buffer_view[start:end]), capture_time
                buffer_view[:buffer_end - end] = buffer_view[end:buffer_end]
                buffer_end -= end

            self.log.debug('Stream ended')
        except httpx.ReadTimeout:
            self.log.warning('Connection to %s timed out', self._url)

    async def run_capture_task(self) -> None:
        self.log.debug('Starting capture task for %s', self._url)

        async with httpx.AsyncClient() as client:
            try:
                await self._prepare_stream()
                async with open_stream(client, self._url, self._username, self._password) as response:
                    if response is not None:
                        async for image, capture_time in self._frame_reader(response):
                            if not image:
                                continue
                            try:
                                timestamp = capture_time if capture_time is not None else rosys.time()
                                array = process_jpeg_image(remove_exif(image), self.rotation, self.crop)
                                if array is None:
                                    continue
                                result = self._on_new_image_data(array, timestamp)
                                if isinstance(result, Awaitable):
                                    await result
                            except Exception as e:
                                self.log.error('Error processing image: %s', e)
            except Exception as e:
                self.log.warning('Connection to %s failed. Was something disconnected?\n%s', self._url, e)
                raise

        self.log.warning('Capture task stopped')
        self._capture_task = None

    def shutdown(self) -> None:
        self.log.debug('Shutting down capture task')
        if self._capture_task is not None:
            self._capture_task.cancel()
            self._capture_task = None

    async def get_fps(self) -> int | None:
        return None

    async def set_fps(self, fps: int) -> None:
        pass

    async def get_resolution(self) -> tuple[int, int] | None:
        return None

    async def set_resolution(self, width: int, height: int) -> None:
        pass

    async def get_mirrored(self) -> bool | None:
        return None

    async def set_mirrored(self, mirrored: bool) -> None:
        pass


def auth_for_challenge(www_authenticate: str, username: str, password: str) -> httpx.Auth:
    """Map a ``WWW-Authenticate`` challenge to the matching httpx auth handler. Fall back to basic if unknown."""
    scheme = www_authenticate.split(' ', 1)[0].lower()
    if scheme == 'digest':
        return httpx.DigestAuth(username, password)
    if scheme != 'basic':
        log.debug('unknown auth scheme "%s", falling back to basic', scheme)
    return httpx.BasicAuth(username, password)


@asynccontextmanager
async def open_stream(client: httpx.AsyncClient, url: str,
                      username: str | None, password: str | None) -> AsyncIterator[httpx.Response | None]:
    """Negotiate the auth scheme and open the http connection.

    Credentials are only sent after the camera has challenged the unauthenticated request with a 401.
    Yields the live 200 response, or ``None`` if the camera refused the connection.
    """
    auth: httpx.Auth | None = None
    while True:
        async with client.stream('GET', url, auth=auth) as response:
            if response.status_code == 401 and auth is None and username is not None and password is not None:
                auth = auth_for_challenge(response.headers.get('www-authenticate', ''), username, password)
                log.debug('camera at %s challenged with 401, retrying with %s', url, type(auth).__name__)
                continue
            if response.status_code != 200:
                auth_scheme = response.request.headers.get('authorization', '<none>').split(' ', 1)[0]
                log.error('could not connect to %s (auth: %s): %s %s',
                          url, auth_scheme, response.status_code, response.reason_phrase)
                yield None
                return
            yield response
            return
