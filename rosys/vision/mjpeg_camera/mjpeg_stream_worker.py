import asyncio
import logging
import shutil
import tempfile
import time
from collections.abc import AsyncIterator, Callable, Generator, Iterable, Iterator
from contextlib import contextmanager
from typing import ClassVar

import httpx
import zmq
import zmq.asyncio

from ...helpers.spawning import SPAWN_CONTEXT
from ..http import new_client
from ..image_processing import decode_jpeg_image, remove_exif
from .stream_channel import (
    EndReason,
    Frame,
    Message,
    StreamEnded,
    decode,
    encode,
)

log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_stream_worker')


class StreamEndedError(Exception):
    """The stream ended for a reason other than the camera closing it."""

    def __init__(self, reason: EndReason, detail: str = '') -> None:
        super().__init__(detail or reason.name)
        self.reason = reason
        self.detail = detail


class MjpegStreamWorker:
    """Handle of the process that reads and decodes one MJPEG stream session."""

    LIVENESS_INTERVAL: ClassVar[float] = 0.2
    """Seconds between checks whether a worker that sends nothing is still alive."""

    FLUSH_TIMEOUT: ClassVar[float] = 0.5
    """Seconds a message sent just before the worker exited may still be on its way."""

    def __init__(self, name: str, url: str, username: str | None, password: str | None) -> None:
        self._loop = asyncio.get_running_loop()
        self.log = logging.getLogger(f'rosys.vision.mjpeg_camera.mjpeg_stream_worker.{name}')
        self._closing = False

        self._directory = tempfile.mkdtemp(prefix='rosys-mjpeg-')  # only our user may reach the socket inside
        self._endpoint = f'ipc://{self._directory}/frames'
        self._socket = zmq.asyncio.Context.instance().socket(zmq.PULL)
        self._socket.set(zmq.CONFLATE, 1)  # keep only the newest message, dropping frames the loop was too busy for
        self._socket.bind(self._endpoint)
        self._process = SPAWN_CONTEXT.Process(target=_run_worker, args=(url, username, password, self._endpoint),
                                              name=f'mjpeg stream {name}', daemon=True)
        self._process.start()

    async def frames(self) -> AsyncIterator[Frame]:
        """Yield frames until the camera closes the stream; raise StreamEndedError for any other end."""
        while True:
            match await self._receive():
                case Frame() as frame:
                    yield frame
                case StreamEnded(reason=EndReason.ENDED):
                    return
                case StreamEnded() as end:
                    raise StreamEndedError(end.reason, end.detail)

    async def shutdown(self) -> None:
        self._closing = True
        if self._process.is_alive():
            self._process.terminate()
            await self._loop.run_in_executor(None, self._process.join, 5.0)
        if self._process.is_alive():
            self.log.warning('stream worker did not end; killing it')
            self._process.kill()
            await self._loop.run_in_executor(None, self._process.join, 1.0)
        self._socket.close()
        shutil.rmtree(self._directory, ignore_errors=True)

    async def _receive(self) -> Message:
        """Wait for the worker's next message, or report that the worker died without sending one."""
        while not self._closing:
            timeout = self.LIVENESS_INTERVAL if self._process.is_alive() else self.FLUSH_TIMEOUT
            if await self._socket.poll(timeout=int(timeout * 1000)):
                return decode((await self._socket.recv(copy=False)).buffer)
            if not self._process.is_alive():
                break
        return self._end_of_stream()

    def _end_of_stream(self) -> StreamEnded:
        if self._closing:
            return StreamEnded(reason=EndReason.ENDED)
        return StreamEnded(reason=EndReason.FAILED,
                           detail=f'the stream worker exited with code {self._process.exitcode}')


def _parse_capture_timestamp(part_header: bytes) -> float | None:
    """Extract the capture instant (Unix epoch seconds) from the ``X-Timestamp`` field of an MJPEG part header."""
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


def _split_frames(chunks: Iterable[bytes]) -> Iterator[tuple[bytes, float | None]]:
    """Yield ``(jpeg, capture_time)`` pairs from the raw bytes of an MJPEG stream."""
    buffer_size = 16 * 1024 * 1024
    buffer = bytearray(buffer_size)
    buffer_view = memoryview(buffer)
    buffer_end = 0

    for chunk in chunks:
        chunk_len = len(chunk)

        if buffer_end + chunk_len > buffer_size:
            log.warning('Buffer overflow, resetting buffer')
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
        capture_time = _parse_capture_timestamp(bytes(buffer_view[:start]))
        end += 2
        yield bytes(buffer_view[start:end]), capture_time
        buffer_view[:buffer_end - end] = buffer_view[end:buffer_end]
        buffer_end -= end


def _auth_for_challenge(www_authenticate: str, username: str, password: str) -> httpx.Auth:
    """Map a ``WWW-Authenticate`` challenge to the matching httpx auth handler. Fall back to basic if unknown."""
    scheme = www_authenticate.split(' ', 1)[0].lower()
    if scheme == 'digest':
        return httpx.DigestAuth(username, password)
    if scheme != 'basic':
        log.debug('unknown auth scheme "%s", falling back to basic', scheme)
    return httpx.BasicAuth(username, password)


@contextmanager
def _open_stream(client: httpx.Client, url: str,
                 username: str | None, password: str | None) -> Generator[httpx.Response, None, None]:
    """Negotiate the auth scheme and open the http connection, yielding the camera's final answer.

    Credentials are only sent after the camera has challenged the unauthenticated request with a 401.
    """
    auth: httpx.Auth | None = None
    while True:
        with client.stream('GET', url, auth=auth) as response:
            if response.status_code == 401 and auth is None and username is not None and password is not None:
                auth = _auth_for_challenge(response.headers.get('www-authenticate', ''), username, password)
                log.debug('camera at %s challenged with 401, retrying with %s', url, type(auth).__name__)
                continue
            yield response
            return


SEND_TIMEOUT: float = 5.0
"""Seconds a message may wait for a parent that is no longer receiving before the worker gives up."""

FAREWELL: float = 5.0
"""Seconds the worker stays around after its last message, waiting to be ended by the parent that reads it."""


def _run_worker(url: str, username: str | None, password: str | None, endpoint: str) -> None:
    """Stream the camera to the parent, which listens on the given endpoint."""
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.set(zmq.SNDTIMEO, int(SEND_TIMEOUT * 1000))
    socket.connect(endpoint)
    # a message sent before the parent's end of the connection is there would be dropped instead of queued
    socket.poll(timeout=int(SEND_TIMEOUT * 1000), flags=zmq.POLLOUT)

    def send(message: Message) -> None:
        try:
            socket.send(encode(message))
        except zmq.Again as e:
            raise BrokenPipeError('the parent is no longer receiving') from e

    try:
        _stream(url, username, password, send)
    finally:
        # a conflating socket drops the message it holds when its sender disconnects,
        # so stay connected until the parent has read the last message and terminates this process
        time.sleep(FAREWELL)
        context.destroy(linger=0)


def _stream(url: str, username: str | None, password: str | None, send: Callable[[Message], None]) -> None:
    try:
        with new_client() as client, _open_stream(client, url, username, password) as response:
            if response.status_code != 200:
                auth_scheme = response.request.headers.get('authorization', '<none>').split(' ', 1)[0]
                detail = f'{response.status_code} {response.reason_phrase} (auth: {auth_scheme})'
                send(StreamEnded(reason=EndReason.REFUSED, detail=detail))
                return
            for jpeg, capture_time in _split_frames(response.iter_bytes()):
                array = decode_jpeg_image(remove_exif(jpeg))
                if array is not None:
                    send(Frame(array=array, capture_time=capture_time))
        send(StreamEnded(reason=EndReason.ENDED))
    except (BrokenPipeError, ConnectionResetError):
        return  # the parent is gone
    except httpx.ReadTimeout:
        send(StreamEnded(reason=EndReason.STALLED))
    except httpx.HTTPError as e:
        send(StreamEnded(reason=EndReason.UNREACHABLE, detail=str(e) or type(e).__name__))
    except Exception as e:  # pylint: disable=broad-exception-caught
        send(StreamEnded(reason=EndReason.FAILED, detail=f'{type(e).__name__}: {e}'))
