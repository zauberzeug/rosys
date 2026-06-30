import asyncio
import logging
import multiprocessing
import os
import threading
from collections.abc import Awaitable, Callable
from multiprocessing.connection import Connection
from multiprocessing.context import SpawnProcess
from typing import Any

from ...geometry import Rectangle
from ..image import BytesImage, Image, ImageArray, MemfdImage
from ..image_rotation import ImageRotation
from .mjpeg_device import MjpegDevice
from .mjpeg_device_factory import MjpegDeviceFactory

# spawn, not fork (which is broken for Python), regardless of the global start method (see path planning, #19)
SPAWN_CONTEXT = multiprocessing.get_context('spawn')

# memfd hands the frame to the main process as a file descriptor instead of copying the pixels through the
# pipe (the main-process bottleneck). Linux-only; elsewhere we fall back to copying via BytesImage.
ImageCarrier = MemfdImage if hasattr(os, 'memfd_create') else BytesImage


class MjpegCaptureProcess(SpawnProcess):  # always spawn
    """Subprocess that owns the MJPEG stream, decodes frames and ships ready ``Image`` objects."""

    def __init__(self, *,
                 camera_id: str,
                 mac: str,
                 ip: str,
                 index: int | None,
                 username: str | None,
                 password: str | None,
                 rotation: ImageRotation,
                 crop: Rectangle | None,
                 parameters: dict[str, Any],
                 image_writer: Connection,
                 control_connection: Connection) -> None:
        super().__init__()
        self._camera_id = camera_id
        self._mac = mac
        self._ip = ip
        self._index = index
        self._username = username
        self._password = password
        self._rotation = rotation
        self._crop = crop
        self._parameters = parameters
        self._image_writer = image_writer
        self._control_connection = control_connection
        self.log = logging.getLogger(f'rosys.vision.mjpeg_camera.{camera_id}.process')
        self._device: MjpegDevice | None = None
        self._stop: asyncio.Event | None = None
        self._tasks: set[asyncio.Task] = set()

    def run(self) -> None:
        try:
            asyncio.run(self._main())
        except (KeyboardInterrupt, EOFError):
            pass

    async def _main(self) -> None:
        # constructing the device starts its capture task on this loop (see MjpegDevice.start_capture_task)
        self._device = MjpegDeviceFactory.create(self._mac, self._ip, index=self._index,
                                                 username=self._username, password=self._password,
                                                 on_new_image_data=self._handle_image_data)
        # set before the first await, so the capture task picks them up on its first frame
        self._device.rotation = self._rotation
        self._device.crop = self._crop
        for name, value in self._parameters.items():
            args = value if isinstance(value, tuple) else (value,)
            await self._call_device(f'set_{name}', args)

        self._stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        loop.add_reader(self._control_connection.fileno(), self._on_control_readable)
        await self._stop.wait()

    def _request_stop(self) -> None:
        if self._stop is not None:
            self._stop.set()

    def _handle_image_data(self, image_array: ImageArray, timestamp: float) -> None:
        carrier = ImageCarrier.from_array(image_array, camera_id=self._camera_id, time=timestamp)
        try:
            self._image_writer.send(carrier)
        except (BrokenPipeError, OSError):
            self._request_stop()

    def _on_control_readable(self) -> None:
        try:
            command = self._control_connection.recv()
        except EOFError:
            self._request_stop()
            return
        if command is None:  # shutdown sentinel
            self._request_stop()
            return
        task = asyncio.create_task(self._handle_command(command))
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    async def _handle_command(self, command: tuple[str, tuple]) -> None:
        method, args = command
        result = await self._call_device(method, args)
        try:
            self._control_connection.send(result)
        except (BrokenPipeError, OSError):
            self._request_stop()

    async def _call_device(self, method: str, args: tuple) -> Any:
        assert self._device is not None
        try:
            result = getattr(self._device, method)(*args)
            if isinstance(result, Awaitable):
                result = await result
            return result
        except Exception as e:  # forwarded to the caller, which re-raises
            self.log.exception('command %s%s failed', method, args)
            return e


class MjpegCapture:
    """Parent-side handle for an :class:`MjpegCaptureProcess`.

    Owns the process boundary: it creates both pipes, spawns the process with the child ends, and keeps the
    parent ends. Decoded images arrive on a reader thread and are delivered onto ``loop`` via ``on_image``;
    ``set_*`` / ``get_*`` requests are forwarded over the control pipe by :meth:`call`.
    """

    def __init__(self, *,
                 camera_id: str,
                 mac: str,
                 ip: str,
                 index: int | None,
                 username: str | None,
                 password: str | None,
                 rotation: ImageRotation,
                 crop: Rectangle | None,
                 parameters: dict[str, Any],
                 loop: asyncio.AbstractEventLoop,
                 on_image: Callable[[Image], None]) -> None:
        self._loop = loop
        self._on_image = on_image
        self._control_lock = threading.Lock()

        self._image_reader, self._image_writer = SPAWN_CONTEXT.Pipe(duplex=False)
        self._control_parent, control_child = SPAWN_CONTEXT.Pipe()
        self._control_connection = control_child
        self._process = MjpegCaptureProcess(camera_id=camera_id, mac=mac, ip=ip, index=index,
                                            username=username, password=password,
                                            rotation=rotation, crop=crop, parameters=parameters,
                                            image_writer=self._image_writer, control_connection=control_child)

    @property
    def is_running(self) -> bool:
        return self._process.is_alive()

    def start(self) -> None:
        self._process.start()
        self._image_writer.close()  # the child holds the only writing end now, so the reader sees EOF on exit
        self._control_connection.close()
        threading.Thread(target=self._read_images, daemon=True).start()

    def _read_images(self) -> None:
        while True:
            try:
                carrier: BytesImage | MemfdImage = self._image_reader.recv()
            except (EOFError, OSError):
                break
            self._loop.call_soon_threadsafe(self._on_image, carrier.to_image())

    async def call(self, method: str, *args: Any) -> Any:
        """Run a device method in the subprocess and return its result (re-raising any exception)."""
        if not self._process.is_alive():
            return None
        return await self._loop.run_in_executor(None, self._call_sync, method, args)

    def _call_sync(self, method: str, args: tuple) -> Any:
        with self._control_lock:
            self._control_parent.send((method, args))
            result = self._control_parent.recv()
        if isinstance(result, BaseException):
            raise result
        return result

    async def shutdown(self) -> None:
        try:
            self._control_parent.send(None)  # shutdown sentinel
        except (BrokenPipeError, OSError):
            pass
        await self._loop.run_in_executor(None, self._process.join, 5.0)
        if self._process.is_alive():
            self._process.terminate()
        self._image_reader.close()
        self._control_parent.close()
