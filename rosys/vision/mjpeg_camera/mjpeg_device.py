import asyncio
import logging
from asyncio import Task
from collections.abc import AsyncGenerator, AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager

import httpx

from ... import rosys
from ...rosys import on_startup
from ..image_processing import remove_exif
from .vendors import mac_to_url


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


def _auth_for_challenge(www_authenticate: str, username: str, password: str) -> httpx.Auth:
    """Map a ``WWW-Authenticate`` challenge to the matching httpx auth handler. Fall back to basic if unknown."""
    scheme = www_authenticate.split(' ', 1)[0].lower()
    if scheme == 'digest':
        return httpx.DigestAuth(username, password)
    return httpx.BasicAuth(username, password)


class MjpegDevice:

    def __init__(self, mac: str, ip: str, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None]) -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device.' + self._mac)

        self._on_new_image_data = on_new_image_data
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
        on_startup(create_capture_task)

    def restart_capture(self) -> None:
        self.log.debug('Restarting capture task')
        self.shutdown()
        self.start_capture_task()

    async def _prepare_stream(self) -> None:
        """Hook executed right before the MJPEG stream is opened (and on every restart).

        Vendors whose HTTP stream must be enabled before it serves data can override this.
        Implementations should log and return on failure rather than raise.
        """

    @asynccontextmanager
    async def _open_stream(self, client: httpx.AsyncClient) -> AsyncIterator[httpx.Response | None]:
        """Open the MJPEG stream, negotiating the auth scheme before any capturing starts.

        Credentials are only sent after the camera has challenged the unauthenticated request with a 401.
        Yields the live 200 response, or ``None`` if the camera refused the connection.
        """
        auth: httpx.Auth | None = None
        while True:
            async with client.stream('GET', self._url, auth=auth) as response:
                if response.status_code == 401 and auth is None \
                        and self._username is not None and self._password is not None:
                    auth = _auth_for_challenge(response.headers.get('www-authenticate', ''),
                                               self._username, self._password)
                    continue
                if response.status_code != 200:
                    auth_scheme = response.request.headers.get('authorization', '<none>').split(' ', 1)[0]
                    self.log.error('could not connect to %s (auth: %s): %s %s',
                                   self._url, auth_scheme, response.status_code, response.reason_phrase)
                    yield None
                    return
                yield response
                return

    async def _frames(self, response: httpx.Response) -> AsyncGenerator[tuple[bytes, float | None], None]:
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
                async with self._open_stream(client) as response:
                    if response is not None:
                        async for image, capture_time in self._frames(response):
                            if not image:
                                continue
                            try:
                                timestamp = capture_time if capture_time is not None else rosys.time()
                                result = self._on_new_image_data(remove_exif(image), timestamp)
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
