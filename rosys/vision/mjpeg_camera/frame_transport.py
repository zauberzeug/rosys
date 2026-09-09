import ctypes
import math
import mmap
import multiprocessing
import os
import sys
from dataclasses import dataclass
from multiprocessing import reduction
from multiprocessing.connection import Connection
from typing import Any, Protocol

import numpy as np

from ..image import ImageArray

# spawn, not fork (which is broken for Python), regardless of the global start method (see path planning, #19)
SPAWN_CONTEXT = multiprocessing.get_context('spawn')


@dataclass(slots=True, kw_only=True)
class Frame:
    array: ImageArray
    capture_time: float | None


class FrameTransport(Protocol):
    """How frames cross the process boundary; any other message crosses pickled through the same connection."""

    def pipe(self) -> tuple[Connection, Connection]:
        """Create the (reader, writer) connection pair."""

    def send_frame(self, connection: Connection, frame: Frame) -> None:
        ...

    def receive(self, connection: Connection) -> Frame | Any:
        """Receive the next message, rebuilding a frame if that is what arrived."""


class PickledFrameTransport:
    """Frames cross as pickled arrays; works everywhere at the cost of copying the pixels twice."""

    def pipe(self) -> tuple[Connection, Connection]:
        return SPAWN_CONTEXT.Pipe(duplex=False)

    def send_frame(self, connection: Connection, frame: Frame) -> None:
        connection.send(frame)

    def receive(self, connection: Connection) -> Frame | Any:
        return connection.recv()


@dataclass(slots=True, kw_only=True)
class _FrameHeader:
    shape: tuple[int, ...]
    capture_time: float | None


def _memfd_create(name: str) -> int:
    if hasattr(os, 'memfd_create'):
        return os.memfd_create(name)  # pylint: disable=no-member
    # some Python builds (e.g. the ones uv installs) lack the os function, while the libc call is there anyway
    libc = ctypes.CDLL(None, use_errno=True)
    fd: int = libc.memfd_create(name.encode(), 0)
    if fd < 0:
        errno = ctypes.get_errno()
        raise OSError(errno, os.strerror(errno))
    return fd


class MemfdFrameTransport:
    """Frames cross as memfd file descriptors, so the receiver maps the pixels instead of copying them."""

    @staticmethod
    def is_available() -> bool:
        if sys.platform != 'linux':
            return False
        try:
            os.close(_memfd_create('rosys-probe'))
        except (OSError, AttributeError):
            return False
        return True

    def pipe(self) -> tuple[Connection, Connection]:
        return SPAWN_CONTEXT.Pipe(duplex=True)  # a socket pair, which can carry file descriptors

    def send_frame(self, connection: Connection, frame: Frame) -> None:
        fd = _memfd_create('rosys-frame')
        try:
            os.write(fd, np.ascontiguousarray(frame.array).data)
            connection.send(_FrameHeader(shape=frame.array.shape, capture_time=frame.capture_time))
            reduction.send_handle(connection, fd, None)
        finally:
            os.close(fd)

    def receive(self, connection: Connection) -> Frame | Any:
        message = connection.recv()
        if not isinstance(message, _FrameHeader):
            return message
        fd = reduction.recv_handle(connection)
        try:
            # populated here so the consumer does not pay the page faults; the array keeps the mapping alive
            buffer = mmap.mmap(fd, math.prod(message.shape), flags=mmap.MAP_SHARED | mmap.MAP_POPULATE)
        finally:
            os.close(fd)
        array = np.frombuffer(buffer, dtype=np.uint8).reshape(message.shape)
        return Frame(array=array, capture_time=message.capture_time)


FRAME_TRANSPORT: FrameTransport = MemfdFrameTransport() if MemfdFrameTransport.is_available() else PickledFrameTransport()
