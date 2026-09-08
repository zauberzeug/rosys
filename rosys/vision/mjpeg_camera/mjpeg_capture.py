import asyncio
import itertools
import logging
import multiprocessing
import threading
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from multiprocessing.connection import Connection
from multiprocessing.context import SpawnProcess
from typing import Any

from nicegui import background_tasks, core

from ...geometry import Rectangle
from ..capture_device import CaptureState
from ..image import Image
from ..image_processing import process_jpeg_image
from ..image_rotation import ImageRotation
from ..reconnect import clamp_reconnect_interval
from .mjpeg_device import MjpegDevice
from .mjpeg_device_factory import MjpegDeviceFactory

# spawn, not fork (which is broken for Python), regardless of the global start method (see path planning, #19)
SPAWN_CONTEXT = multiprocessing.get_context('spawn')


@dataclass(slots=True, kw_only=True)
class DeviceStatus:
    """What the capture process reports about its device."""
    state: CaptureState = CaptureState.CONNECTING
    url: str | None = None


@dataclass(slots=True, kw_only=True)
class _Call:
    id: int
    method: str
    args: tuple


@dataclass(slots=True, kw_only=True)
class _Reply:
    id: int
    result: Any


@dataclass(slots=True, kw_only=True)
class _Set:
    name: str
    value: Any


class _PipeLogHandler(logging.Handler):
    """Ship log records to the parent process, which logs them as its own."""

    def __init__(self, connection: Connection) -> None:
        super().__init__()
        self._connection = connection

    def emit(self, record: logging.LogRecord) -> None:
        record.msg = self.format(record)  # renders the traceback, which does not pickle
        record.args = None
        record.exc_info = None
        record.exc_text = None
        record.stack_info = None
        try:
            self._connection.send(record)
        except (BrokenPipeError, OSError):
            pass


class MjpegCaptureProcess(SpawnProcess):
    """Subprocess that keeps an MJPEG stream alive, decodes its frames and ships ready images."""

    STATUS_INTERVAL = 0.1

    def __init__(self, *,
                 camera_id: str,
                 mac: str,
                 ip: str | None,
                 index: int | None,
                 username: str | None,
                 password: str | None,
                 rotation: ImageRotation,
                 crop: Rectangle | None,
                 parameters: dict[str, Any],
                 reconnect_interval: float,
                 image_writer: Connection,
                 control_connection: Connection) -> None:
        super().__init__(name=f'mjpeg capture {camera_id}')
        self._camera_id = camera_id
        self._mac = mac
        self._ip = ip
        self._index = index
        self._username = username
        self._password = password
        self._rotation = rotation
        self._crop = crop
        self._parameters = parameters
        self._reconnect_interval = reconnect_interval
        self._image_writer = image_writer
        self._control_connection = control_connection
        self._device: MjpegDevice | None = None
        self._stop: asyncio.Event | None = None
        self._status = DeviceStatus()
        self._tasks: set[asyncio.Task] = set()

    def run(self) -> None:
        logging.getLogger().addHandler(_PipeLogHandler(self._image_writer))
        logging.getLogger('rosys').setLevel(logging.INFO)
        try:
            asyncio.run(self._main())
        except (KeyboardInterrupt, EOFError):
            pass

    async def _main(self) -> None:
        core.loop = asyncio.get_running_loop()  # CaptureDevice runs its loop as a nicegui background task
        self._device = MjpegDeviceFactory.create(self._mac, self._ip, index=self._index,
                                                 username=self._username, password=self._password,
                                                 on_new_image_data=self._handle_image_data,
                                                 on_connect=self._publish_status,
                                                 reconnect_interval=self._reconnect_interval)
        await self._apply_parameters()
        self._stop = asyncio.Event()
        core.loop.add_reader(self._control_connection.fileno(), self._on_control_readable)
        publisher = asyncio.create_task(self._publish_status_periodically())
        try:
            await self._stop.wait()
        finally:
            publisher.cancel()
            await self._device.shutdown()

    async def _apply_parameters(self) -> None:
        assert self._device is not None
        for name, value in self._parameters.items():
            if value is None:
                continue
            args = value if isinstance(value, tuple) else (value,)
            await getattr(self._device, f'set_{name}')(*args)

    def _request_stop(self) -> None:
        if self._stop is not None:
            self._stop.set()

    def _send(self, message: Image | DeviceStatus) -> None:
        try:
            self._image_writer.send(message)
        except (BrokenPipeError, OSError):
            self._request_stop()

    def _handle_image_data(self, image_bytes: bytes, timestamp: float) -> None:
        array = process_jpeg_image(image_bytes, self._rotation, self._crop)
        if array is None:
            return
        self._send(Image.from_array(array, camera_id=self._camera_id, time=timestamp))

    async def _publish_status_periodically(self) -> None:
        while True:
            self._publish_status()
            await asyncio.sleep(self.STATUS_INTERVAL)

    def _publish_status(self) -> None:
        assert self._device is not None
        if not self._device.is_active:
            self._request_stop()
        status = DeviceStatus(state=self._device.state, url=self._device.url)
        if status != self._status:
            self._status = status
            self._send(status)

    def _on_control_readable(self) -> None:
        try:
            command = self._control_connection.recv()
        except EOFError:
            self._request_stop()
            return
        if command is None:
            self._request_stop()
        elif isinstance(command, _Set):
            self._apply_setting(command)
        else:
            task = asyncio.create_task(self._handle_call(command))
            self._tasks.add(task)
            task.add_done_callback(self._tasks.discard)

    def _apply_setting(self, setting: _Set) -> None:
        assert self._device is not None
        match setting.name:
            case 'rotation':
                self._rotation = setting.value
            case 'crop':
                self._crop = setting.value
            case 'ip':
                self._device.ip = setting.value
            case 'reconnect_interval':
                self._device.reconnect_interval = setting.value
            case _:
                raise ValueError(f'unknown setting "{setting.name}"')

    async def _handle_call(self, call: _Call) -> None:
        assert self._device is not None
        try:
            result = getattr(self._device, call.method)(*call.args)
            if isinstance(result, Awaitable):
                result = await result
        except Exception as e:  # pylint: disable=broad-exception-caught
            result = RuntimeError(f'{type(e).__name__}: {e}')  # the original may not pickle
        try:
            self._control_connection.send(_Reply(id=call.id, result=result))
        except (BrokenPipeError, OSError):
            self._request_stop()


class MjpegCapture:
    """Parent-side handle of an `MjpegCaptureProcess`, mirroring the state its device reports."""

    CALL_TIMEOUT = 10.0
    """Seconds a forwarded device call may take before it fails."""

    def __init__(self, *,
                 camera_id: str,
                 mac: str,
                 ip: str | None,
                 index: int | None,
                 username: str | None,
                 password: str | None,
                 rotation: ImageRotation,
                 crop: Rectangle | None,
                 parameters: dict[str, Any],
                 reconnect_interval: float,
                 on_image: Callable[[Image], None],
                 on_connect: Callable[[], Awaitable | None] | None = None) -> None:
        self._loop = asyncio.get_running_loop()
        self._on_image = on_image
        self._on_connect = on_connect
        self.log = logging.getLogger(f'rosys.vision.mjpeg_camera.{camera_id}.capture')
        self._ip = ip
        self._rotation = rotation
        self._crop = crop
        self._reconnect_interval = clamp_reconnect_interval(reconnect_interval, self.log)
        self._status = DeviceStatus()
        self._call_ids = itertools.count()
        self._call_lock = threading.Lock()
        self._send_lock = threading.Lock()

        self._image_reader, image_writer = SPAWN_CONTEXT.Pipe(duplex=False)
        self._control, control_child = SPAWN_CONTEXT.Pipe()
        self._process = MjpegCaptureProcess(camera_id=camera_id, mac=mac, ip=ip, index=index,
                                            username=username, password=password,
                                            rotation=rotation, crop=crop, parameters=parameters,
                                            reconnect_interval=self._reconnect_interval,
                                            image_writer=image_writer, control_connection=control_child)
        self._process.start()
        image_writer.close()  # the child holds the only writing end now, so the reader sees EOF on exit
        control_child.close()
        threading.Thread(target=self._read_messages, daemon=True, name=f'mjpeg reader {camera_id}').start()

    @property
    def pid(self) -> int | None:
        return self._process.pid

    @property
    def is_active(self) -> bool:
        """Whether the capture process, and with it the self-healing capture loop, is alive."""
        return self._process.is_alive()

    @property
    def is_connected(self) -> bool:
        return self.is_active and self._status.state is CaptureState.STREAMING

    @property
    def is_refused(self) -> bool:
        return self.is_active and self._status.state is CaptureState.REFUSED

    @property
    def url(self) -> str | None:
        return self._status.url

    @property
    def ip(self) -> str | None:
        return self._ip

    @ip.setter
    def ip(self, ip: str | None) -> None:
        self._ip = ip
        self._send(_Set(name='ip', value=ip))

    @property
    def reconnect_interval(self) -> float:
        return self._reconnect_interval

    @reconnect_interval.setter
    def reconnect_interval(self, interval: float) -> None:
        self._reconnect_interval = clamp_reconnect_interval(interval, self.log)
        self._send(_Set(name='reconnect_interval', value=self._reconnect_interval))

    @property
    def rotation(self) -> ImageRotation:
        return self._rotation

    @rotation.setter
    def rotation(self, rotation: ImageRotation) -> None:
        self._rotation = rotation
        self._send(_Set(name='rotation', value=rotation))

    @property
    def crop(self) -> Rectangle | None:
        return self._crop

    @crop.setter
    def crop(self, crop: Rectangle | None) -> None:
        self._crop = crop
        self._send(_Set(name='crop', value=crop))

    def _send(self, message: _Call | _Set | None) -> None:
        with self._send_lock:
            try:
                self._control.send(message)
            except (BrokenPipeError, OSError):
                pass  # the process is gone, which `is_active` reports

    def _read_messages(self) -> None:
        while True:
            try:
                message = self._image_reader.recv()
            except (EOFError, OSError):
                return
            try:
                self._loop.call_soon_threadsafe(self._handle_message, message)
            except RuntimeError:
                return  # the loop is closed

    def _handle_message(self, message: Image | DeviceStatus | logging.LogRecord) -> None:
        if isinstance(message, Image):
            self._on_image(message)
        elif isinstance(message, DeviceStatus):
            connected = message.state is CaptureState.STREAMING and self._status.state is not CaptureState.STREAMING
            self._status = message
            if connected and self._on_connect is not None:
                background_tasks.create(self._run_on_connect(), name=f'on_connect {self._process.name}')
        else:
            logging.getLogger(message.name).handle(message)

    async def _run_on_connect(self) -> None:
        assert self._on_connect is not None
        try:
            result = self._on_connect()
            if isinstance(result, Awaitable):
                await result
        except Exception as e:  # pylint: disable=broad-exception-caught
            self.log.warning('on_connect callback failed: %s: %s', type(e).__name__, e)

    async def call(self, method: str, *args: Any) -> Any:
        """Run a device method in the capture process and return its result, re-raising its exception."""
        return await self._loop.run_in_executor(None, self._call_sync, method, args)

    def _call_sync(self, method: str, args: tuple) -> Any:
        if not self._process.is_alive():
            raise ConnectionError(f'cannot call {method}: the capture process is not running')
        with self._call_lock:
            call = _Call(id=next(self._call_ids), method=method, args=args)
            self._send(call)
            while True:
                if not self._control.poll(self.CALL_TIMEOUT):
                    raise TimeoutError(f'{method} did not answer within {self.CALL_TIMEOUT} s')
                reply: _Reply = self._control.recv()
                if reply.id == call.id:  # a reply to a call that timed out is dropped
                    break
        if isinstance(reply.result, Exception):
            raise reply.result
        return reply.result

    async def shutdown(self) -> None:
        """Stop the capture process, killing it if it does not end on its own."""
        self._send(None)
        await self._loop.run_in_executor(None, self._process.join, 5.0)
        if self._process.is_alive():
            self.log.warning('capture process did not end; killing it')
            self._process.kill()
            await self._loop.run_in_executor(None, self._process.join, 1.0)
        self._image_reader.close()
        self._control.close()
