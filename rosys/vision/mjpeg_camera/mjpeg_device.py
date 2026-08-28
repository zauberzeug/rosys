import logging
from collections.abc import AsyncGenerator, AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager

import httpx

from ... import rosys
from ..capture_device import CaptureDevice, CaptureState
from ..http import new_async_client
from ..image_processing import remove_exif
from .vendors import mac_to_url

log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device')


class CameraAddressUnknown(Exception):
    """Raised when the camera settings are used before discovery has found an address."""


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


class MjpegDevice(CaptureDevice):

    def __init__(self, mac: str, ip: str | None = None, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None],
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 reconnect_interval: float = 3.0) -> None:
        super().__init__(name=mac,
                         log=logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device.' + mac),
                         reconnect_interval=reconnect_interval)
        self._mac = mac
        self._ip = ip
        self._index = index
        self._on_new_image_data = on_new_image_data
        self._on_connect = on_connect
        self._username = username
        self._password = password

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

    def _retry_reason(self) -> str | None:
        if self._ip is None:
            return 'no address known'
        if self.url is None:
            return f'no stream URL for mac "{self._mac}"'
        if self.is_refused:
            return 'camera refused the stream'
        return None

    def _describe_session_error(self, error: Exception) -> str:
        if isinstance(error, CameraAddressUnknown):
            return 'no address known yet'
        if isinstance(error, httpx.HTTPError):
            return f'cannot reach the camera: {error}'
        return super()._describe_session_error(error)

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

    async def _run_session(self) -> bool:
        """Open a stream and read it until it ends. Returns whether at least one frame arrived."""
        streamed = False
        url = self.url
        if url is None:
            return streamed
        self.log.debug('Starting capture task for %s', url)

        async with new_async_client() as client:
            await self._prepare_stream()
            async with open_stream(client, url, self._username, self._password) as response:
                if response is None:
                    self._set_state(CaptureState.REFUSED)
                    return streamed
                self._set_state(CaptureState.STREAMING)
                await self._invoke_on_connect()
                async for image, capture_time in self._frame_reader(response):
                    if self.url != url:
                        self.log.info('stream settings changed; reopening the stream')
                        return streamed
                    if not image:
                        continue
                    streamed = True
                    try:
                        timestamp = capture_time if capture_time is not None else rosys.time()
                        callback_result = self._on_new_image_data(remove_exif(image), timestamp)
                        if isinstance(callback_result, Awaitable):
                            await callback_result
                    except Exception as e:
                        self.log.error('Error processing image: %s', e)
                    if not self._keeps_running():
                        return streamed
        self.log.debug('capture session ended')
        return streamed

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
    Yields the live 200 response, or ``None`` when the camera answered something else.
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
                log.error('camera at %s refused the stream (auth: %s): %s %s',
                          url, auth_scheme, response.status_code, response.reason_phrase)
                yield None
                return
            yield response
            return
