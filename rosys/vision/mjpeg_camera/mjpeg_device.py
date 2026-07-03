import asyncio
import logging
from asyncio import Task
from collections.abc import AsyncGenerator, Awaitable, Callable

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
        self._authentication = None if username is None or password is None else httpx.DigestAuth(username, password)
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

    async def run_capture_task(self) -> None:
        self.log.debug('Starting capture task for %s', self._url)

        async def stream() -> AsyncGenerator[tuple[bytes, float | None], None]:
            async with httpx.AsyncClient() as client:
                assert self._url is not None
                try:
                    await self._prepare_stream()
                    async with client.stream('GET', self._url, auth=self._authentication) as response:  # type: ignore
                        if response.status_code == 401:
                            self.log.error('unauthorized (401) for %s (credentials: %s); giving up',
                                           self._url, self._authentication)
                            self._authorized = False
                            return
                        if response.status_code != 200:
                            self.log.error('could not connect to %s (credentials: %s): %s %s',
                                           self._url, self._authentication, response.status_code, response.reason_phrase)
                            return
                        self._streaming = True

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
                except Exception as e:
                    self.log.warning('Connection to %s failed. Was something disconnected?\n%s', self._url, e)
                    raise e

        try:
            async for image, capture_time in stream():
                if not image:
                    continue
                try:
                    timestamp = capture_time if capture_time is not None else rosys.time()
                    result = self._on_new_image_data(remove_exif(image), timestamp)
                    if isinstance(result, Awaitable):
                        await result
                except Exception as e:
                    self.log.error('Error processing image: %s', e)
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
