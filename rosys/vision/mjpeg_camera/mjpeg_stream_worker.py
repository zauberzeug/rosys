import asyncio
import enum
import logging
import multiprocessing
import threading
from collections import deque
from collections.abc import Generator, Iterable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from multiprocessing.connection import Connection

import httpx

from ..http import new_client
from ..image import ImageArray
from ..image_processing import decode_jpeg_image, remove_exif

# spawn, not fork (which is broken for Python), regardless of the global start method (see path planning, #19)
SPAWN_CONTEXT = multiprocessing.get_context('spawn')

log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_stream_worker')


class EndReason(enum.Enum):
    ENDED = enum.auto()
    REFUSED = enum.auto()  # the camera answered something other than a stream
    UNREACHABLE = enum.auto()  # http error
    FAILED = enum.auto()  # anything else, including the worker dying


@dataclass(slots=True, kw_only=True)
class StartStream:
    session: int
    url: str
    username: str | None
    password: str | None


@dataclass(slots=True)
class StopStream:
    pass


@dataclass(slots=True, kw_only=True)
class StreamOpened:
    session: int


@dataclass(slots=True, kw_only=True)
class Frame:
    session: int
    array: ImageArray
    capture_time: float | None


@dataclass(slots=True, kw_only=True)
class StreamEnded:
    session: int
    reason: EndReason
    detail: str = ''


def parse_capture_timestamp(part_header: bytes) -> float | None:
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


def split_frames(chunks: Iterable[bytes]) -> Iterator[tuple[bytes, float | None]]:
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
        capture_time = parse_capture_timestamp(bytes(buffer_view[:start]))
        end += 2
        yield bytes(buffer_view[start:end]), capture_time
        buffer_view[:buffer_end - end] = buffer_view[end:buffer_end]
        buffer_end -= end


def auth_for_challenge(www_authenticate: str, username: str, password: str) -> httpx.Auth:
    """Map a ``WWW-Authenticate`` challenge to the matching httpx auth handler. Fall back to basic if unknown."""
    scheme = www_authenticate.split(' ', 1)[0].lower()
    if scheme == 'digest':
        return httpx.DigestAuth(username, password)
    if scheme != 'basic':
        log.debug('unknown auth scheme "%s", falling back to basic', scheme)
    return httpx.BasicAuth(username, password)


@contextmanager
def open_stream(client: httpx.Client, url: str,
                username: str | None, password: str | None) -> Generator[httpx.Response, None, None]:
    """Negotiate the auth scheme and open the http connection, yielding the camera's final answer.

    Credentials are only sent after the camera has challenged the unauthenticated request with a 401.
    """
    auth: httpx.Auth | None = None
    while True:
        with client.stream('GET', url, auth=auth) as response:
            if response.status_code == 401 and auth is None and username is not None and password is not None:
                auth = auth_for_challenge(response.headers.get('www-authenticate', ''), username, password)
                log.debug('camera at %s challenged with 401, retrying with %s', url, type(auth).__name__)
                continue
            yield response
            return


class _ParentGone(Exception):
    pass


def _run_worker(control: Connection, output: Connection) -> None:
    def send(message: StreamOpened | Frame | StreamEnded) -> None:
        try:
            output.send(message)
        except (BrokenPipeError, OSError) as e:
            raise _ParentGone() from e

    with new_client() as client:
        while True:
            try:
                command = control.recv()
            except EOFError:
                return
            if command is None:
                return
            if isinstance(command, StartStream):
                try:
                    _stream(client, command, control, send)
                except _ParentGone:
                    return


def _stream(client: httpx.Client, start: StartStream, control: Connection, send) -> None:
    session = start.session
    try:
        with open_stream(client, start.url, start.username, start.password) as response:
            if response.status_code != 200:
                auth_scheme = response.request.headers.get('authorization', '<none>').split(' ', 1)[0]
                detail = f'{response.status_code} {response.reason_phrase} (auth: {auth_scheme})'
                send(StreamEnded(session=session, reason=EndReason.REFUSED, detail=detail))
                return
            send(StreamOpened(session=session))
            for jpeg, capture_time in split_frames(response.iter_bytes()):
                if control.poll():  # a stop or shutdown is pending; the main loop picks it up
                    return
                array = decode_jpeg_image(remove_exif(jpeg))
                if array is None:
                    continue
                send(Frame(session=session, array=array, capture_time=capture_time))
        send(StreamEnded(session=session, reason=EndReason.ENDED))
    except _ParentGone:
        raise
    except httpx.HTTPError as e:
        send(StreamEnded(session=session, reason=EndReason.UNREACHABLE, detail=str(e) or type(e).__name__))
    except Exception as e:  # pylint: disable=broad-exception-caught
        send(StreamEnded(session=session, reason=EndReason.FAILED, detail=f'{type(e).__name__}: {e}'))


class MjpegStreamWorker:
    """Handle of the process that reads and decodes one camera's MJPEG stream, one session at a time."""

    def __init__(self, name: str) -> None:
        self._loop = asyncio.get_running_loop()
        self.log = logging.getLogger(f'rosys.vision.mjpeg_camera.mjpeg_stream_worker.{name}')
        self._session = 0
        self._messages: deque[StreamOpened | Frame | StreamEnded] = deque()
        self._message_arrived = asyncio.Event()

        self._control, control_child = SPAWN_CONTEXT.Pipe()
        self._output, output_child = SPAWN_CONTEXT.Pipe(duplex=False)
        self._process = SPAWN_CONTEXT.Process(target=_run_worker, args=(control_child, output_child),
                                              name=f'mjpeg stream {name}', daemon=True)
        self._process.start()
        control_child.close()
        output_child.close()  # the child holds the only writing end now, so the reader sees EOF when it exits
        threading.Thread(target=self._read_messages, daemon=True, name=f'mjpeg stream reader {name}').start()

    @property
    def is_alive(self) -> bool:
        return self._process.is_alive()

    def start_stream(self, url: str, username: str | None, password: str | None) -> None:
        """Open the stream in the worker; messages of an earlier session are discarded from now on."""
        self._session += 1
        self._messages.clear()
        self._send(StartStream(session=self._session, url=url, username=username, password=password))

    def stop_stream(self) -> None:
        self._send(StopStream())

    async def receive(self) -> StreamOpened | Frame | StreamEnded:
        while not self._messages:
            self._message_arrived.clear()
            await self._message_arrived.wait()
        return self._messages.popleft()

    async def shutdown(self) -> None:
        self._send(None)
        await self._loop.run_in_executor(None, self._process.join, 5.0)
        if self._process.is_alive():
            self.log.warning('stream worker did not end; killing it')
            self._process.kill()
            await self._loop.run_in_executor(None, self._process.join, 1.0)
        self._control.close()
        self._output.close()

    def _send(self, command: StartStream | StopStream | None) -> None:
        try:
            self._control.send(command)
        except (BrokenPipeError, OSError):
            pass  # the worker is gone, which the reader thread reports as the end of the stream

    def _read_messages(self) -> None:
        while True:
            try:
                message = self._output.recv()
            except (EOFError, OSError):
                message = None
            try:
                self._loop.call_soon_threadsafe(self._offer, message)
            except RuntimeError:
                return  # the loop is closed
            if message is None:
                return

    def _offer(self, message: StreamOpened | Frame | StreamEnded | None) -> None:
        if message is None:
            message = StreamEnded(session=self._session, reason=EndReason.FAILED, detail='the stream worker exited')
        elif message.session != self._session:
            return
        if isinstance(message, Frame) and self._messages and isinstance(self._messages[-1], Frame):
            self._messages[-1] = message  # a frame nobody has picked up yet is stale
        else:
            self._messages.append(message)
        self._message_arrived.set()
