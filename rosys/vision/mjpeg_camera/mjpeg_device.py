import asyncio
import enum
import logging
from asyncio import Task
from collections.abc import AsyncGenerator, AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import ClassVar

import httpx
from nicegui import background_tasks

from ... import rosys
from ..image_processing import remove_exif
from .vendors import mac_to_url

log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device')


class CaptureState(enum.Enum):
    """State of the self-healing capture loop.

    Modelling the loop as a single state (instead of separate ``should_run``/``streaming``/``authorized``
    flags) keeps only the valid combinations representable.
    """
    CONNECTING = enum.auto()    # loop alive, opening the stream or waiting to reconnect
    STREAMING = enum.auto()     # stream open and delivering frames
    UNAUTHORIZED = enum.auto()  # camera rejected our credentials; loop backs off before trying again
    STOPPED = enum.auto()       # shutdown requested; loop gives up


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
    UNAUTHORIZED_RECONNECT_INTERVAL: ClassVar[float] = 60.0
    '''Wait time between attempts while the camera rejects our credentials.'''

    def __init__(self, mac: str, ip: str | None = None, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None],
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 reconnect_interval: float = 3.0) -> None:
        self._mac = mac
        self._ip = ip
        self._index = index
        self.log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device.' + self._mac)

        self._on_new_image_data = on_new_image_data
        self._on_connect = on_connect
        self._capture_task: Task | None = None
        self._username = username
        self._password = password
        self.reconnect_interval = reconnect_interval
        self._state = CaptureState.CONNECTING

        self._start_capture_task()

    @property
    def ip(self) -> str | None:
        """The address of the camera; assigning a new one makes the capture loop reopen the stream there."""
        return self._ip

    @ip.setter
    def ip(self, ip: str | None) -> None:
        if ip == self._ip:
            return
        self.log.info('address changed to %s', ip)
        self._ip = ip
        self.restart_capture()

    @property
    def url(self) -> str | None:
        """The stream URL, or ``None`` when the address is unknown or no URL scheme is known for the mac."""
        if self._ip is None:
            return None
        return mac_to_url(self._mac, self._ip, index=self._index)

    @property
    def is_connected(self) -> bool:
        """Whether the MJPEG stream is currently delivering frames."""
        return self._state is CaptureState.STREAMING

    @property
    def is_active(self) -> bool:
        """Whether the self-healing capture loop is alive (streaming or waiting to reconnect)."""
        return (self._capture_task is not None) and (not self._capture_task.done())

    @property
    def authorized(self) -> bool:
        """Whether the last attempt was not rejected; ``False`` while backing off after a 401."""
        return self._state is not CaptureState.UNAUTHORIZED

    def _start_capture_task(self) -> None:
        if self._capture_task is not None and not self._capture_task.done():
            self.log.warning('tried to start a capture loop while already running one')
            return
        self._state = CaptureState.CONNECTING
        self._capture_task = background_tasks.create(self._run_capture_task(), name=f'mjpeg capture {self._mac}')

    def _keeps_running(self) -> bool:
        """Whether the calling capture task should carry on.

        A shutdown ends it, and so does a restart: `_capture_task` then points at the new task while
        the old one may still be resuming from an await that swallowed its cancellation, because
        `rosys.run.cpu_bound` and `io_bound` return ``None`` instead of raising `CancelledError`.
        """
        return self._state is not CaptureState.STOPPED and self._capture_task is asyncio.current_task()

    async def _run_capture_task(self) -> None:
        """Keep a single MJPEG session alive, reconnecting after `reconnect_interval` when it ends.

        Runs until `shutdown()` cancels the task.
        """
        try:
            while self._keeps_running():
                # every attempt starts fresh: an earlier rejection says nothing about this one
                self._state = CaptureState.CONNECTING
                try:
                    await self._connect_and_stream_images()
                except Exception:
                    self.log.exception('capture session failed')
                if not self._keeps_running():
                    break
                if self._state is CaptureState.UNAUTHORIZED:
                    self.log.info('credentials rejected; retrying in %.1f s', self._retry_interval)
                elif self._ip is None:
                    self.log.debug('no address known; retrying in %.1f s', self._retry_interval)
                elif self.url is None:
                    self.log.debug('no stream URL for mac "%s"; retrying in %.1f s',
                                   self._mac, self._retry_interval)
                else:
                    self.log.info('stream ended; reconnecting in %.1f s', self._retry_interval)
                # a new address restarts the whole task, so no need to cut this wait short
                await rosys.sleep(self._retry_interval)
        finally:
            if self._capture_task is asyncio.current_task():
                self._capture_task = None

    @property
    def _retry_interval(self) -> float:
        """How long to wait before the next session; a rejected login backs off further than a lost stream."""
        if self._state is CaptureState.UNAUTHORIZED:
            return self.UNAUTHORIZED_RECONNECT_INTERVAL
        return self.reconnect_interval

    def restart_capture(self) -> None:
        self.log.debug('Restarting capture task')
        self.shutdown()
        self._start_capture_task()

    async def _invoke_on_connect(self) -> None:
        """Notify the owner that a capture session has been (re-)established, e.g. to reapply camera parameters."""
        if self._on_connect is None:
            return
        result = self._on_connect()
        if isinstance(result, Awaitable):
            await result

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
            self.log.warning('Connection to %s timed out', self.url)

    async def _connect_and_stream_images(self) -> None:
        url = self.url
        if url is None:
            return
        self.log.debug('Starting capture task for %s', url)

        try:
            async with httpx.AsyncClient() as client:
                try:
                    await self._prepare_stream()
                    async with open_stream(client, url, self._username, self._password) as result:
                        if result.unauthorized:
                            self._state = CaptureState.UNAUTHORIZED
                            return
                        if result.response is None:
                            return
                        self._state = CaptureState.STREAMING
                        await self._invoke_on_connect()
                        async for image, capture_time in self._frame_reader(result.response):
                            if self.url != url:
                                self.log.info('stream settings changed; reopening the stream')
                                return
                            if not image:
                                continue
                            try:
                                timestamp = capture_time if capture_time is not None else rosys.time()
                                callback_result = self._on_new_image_data(remove_exif(image), timestamp)
                                if isinstance(callback_result, Awaitable):
                                    await callback_result
                            except Exception as e:
                                self.log.error('Error processing image: %s', e)
                            if not self._keeps_running():
                                return
                except Exception as e:
                    self.log.warning('Connection to %s failed. Was something disconnected?\n%s', url, e)
                    raise
        finally:
            if self._keeps_running() and self._state is CaptureState.STREAMING:
                self._state = CaptureState.CONNECTING
        self.log.debug('capture session ended')

    def shutdown(self) -> None:
        self.log.debug('Shutting down capture task')
        self._state = CaptureState.STOPPED
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
