import asyncio
import logging
from asyncio import Task
from collections.abc import AsyncGenerator, AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass

import httpx

from ... import rosys
from ...rosys import on_startup
from ..image_processing import remove_exif
from .vendors import mac_to_url

log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device')


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
                 on_new_image_data: Callable[[bytes, float], Awaitable | None],
                 reconnect_interval: float = 3.0) -> None:
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
        self.reconnect_interval = reconnect_interval
        self._should_run: bool = True
        self._streaming: bool = False
        self._authorized: bool = True

        self.start_capture_task()

    @property
    def is_connected(self) -> bool:
        """Whether the MJPEG stream is currently delivering frames."""
        return self._streaming

    @property
    def is_active(self) -> bool:
        """Whether the self-healing capture loop is alive (streaming or waiting to reconnect)."""
        return (self._capture_task is not None) and (not self._capture_task.done())

    @property
    def authorized(self) -> bool:
        return self._authorized

    def start_capture_task(self) -> None:
        def create_capture_task() -> None:
            if self._capture_task is not None and not self._capture_task.done():
                return
            self._should_run = True
            loop = asyncio.get_event_loop()
            self._capture_task = loop.create_task(self._run_capture_loop())
        on_startup(create_capture_task)

    async def _run_capture_loop(self) -> None:
        """Keep a single MJPEG session alive, reconnecting after `reconnect_interval` when it ends.

        Runs until `shutdown()` cancels the task or the camera rejects us as unauthorized.
        """
        try:
            while self._should_run:
                try:
                    await self.run_capture_task()
                except Exception:
                    self.log.exception('capture session failed')
                if not self._should_run or not self._authorized:
                    break
                self.log.info('stream ended; reconnecting in %.1f s', self.reconnect_interval)
                await rosys.sleep(self.reconnect_interval)
        finally:
            if self._capture_task is asyncio.current_task():
                self._capture_task = None

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

        try:
            async with httpx.AsyncClient() as client:
                try:
                    await self._prepare_stream()
                    async with open_stream(client, self._url, self._username, self._password) as result:
                        if result.unauthorized:
                            self._authorized = False
                            return
                        if result.response is None:
                            return
                        self._streaming = True
                        async for image, capture_time in self._frame_reader(result.response):
                            if not image:
                                continue
                            try:
                                timestamp = capture_time if capture_time is not None else rosys.time()
                                callback_result = self._on_new_image_data(remove_exif(image), timestamp)
                                if isinstance(callback_result, Awaitable):
                                    await callback_result
                            except Exception as e:
                                self.log.error('Error processing image: %s', e)
                except Exception as e:
                    self.log.warning('Connection to %s failed. Was something disconnected?\n%s', self._url, e)
                    raise
        finally:
            self._streaming = False
        self.log.debug('capture session ended')

    def shutdown(self) -> None:
        self.log.debug('Shutting down capture task')
        self._should_run = False
        self._streaming = False
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


@dataclass(frozen=True, slots=True)
class StreamOpenResult:
    response: httpx.Response | None
    unauthorized: bool = False


@asynccontextmanager
async def open_stream(client: httpx.AsyncClient, url: str,
                      username: str | None, password: str | None) -> AsyncIterator[StreamOpenResult]:
    """Negotiate the auth scheme and open the http connection.

    Credentials are only sent after the camera has challenged the unauthenticated request with a 401.
    Yields a :class:`StreamOpenResult` with the live 200 response, or ``None`` if the camera refused the connection.
    """
    auth: httpx.Auth | None = None
    while True:
        async with client.stream('GET', url, auth=auth) as response:
            if response.status_code == 401:
                if auth is None and username is not None and password is not None:
                    auth = auth_for_challenge(response.headers.get('www-authenticate', ''), username, password)
                    log.debug('camera at %s challenged with 401, retrying with %s', url, type(auth).__name__)
                    continue
                auth_scheme = response.request.headers.get('authorization', '<none>').split(' ', 1)[0]
                log.error('unauthorized (401) for %s (auth: %s)', url, auth_scheme)
                yield StreamOpenResult(None, unauthorized=True)
                return
            if response.status_code != 200:
                auth_scheme = response.request.headers.get('authorization', '<none>').split(' ', 1)[0]
                log.error('could not connect to %s (auth: %s): %s %s',
                          url, auth_scheme, response.status_code, response.reason_phrase)
                yield StreamOpenResult(None)
                return
            yield StreamOpenResult(response)
            return
