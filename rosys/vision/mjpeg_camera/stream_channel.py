import ctypes
import enum
import math
import mmap
import multiprocessing
import os
import sys
from dataclasses import dataclass
from multiprocessing import reduction
from multiprocessing.connection import Connection
from typing import Protocol

import numpy as np

from ..image import ImageArray

# spawn, not fork (which is broken for Python), regardless of the global start method (see path planning, #19)
SPAWN_CONTEXT = multiprocessing.get_context('spawn')


class EndReason(enum.Enum):
    ENDED = enum.auto()
    REFUSED = enum.auto()  # the camera answered something other than a stream
    UNREACHABLE = enum.auto()  # http error
    FAILED = enum.auto()  # anything else, including the worker dying


@dataclass(slots=True)
class StreamOpened:
    pass


@dataclass(slots=True, kw_only=True)
class Frame:
    array: ImageArray
    capture_time: float | None


@dataclass(slots=True, kw_only=True)
class StreamEnded:
    reason: EndReason
    detail: str = ''


Message = StreamOpened | Frame | StreamEnded


class MessageSender(Protocol):

    def send(self, message: Message) -> None:
        ...

    def close(self) -> None:
        ...


class MessageReceiver(Protocol):

    def receive(self) -> Message:
        ...

    def close(self) -> None:
        ...


class PickledSender:
    """Frames cross as pickled arrays; works everywhere at the cost of copying the pixels twice."""

    def __init__(self, connection: Connection) -> None:
        self._connection = connection

    def send(self, message: Message) -> None:
        self._connection.send(message)

    def close(self) -> None:
        self._connection.close()


class PickledReceiver:

    def __init__(self, connection: Connection) -> None:
        self._connection = connection

    def receive(self) -> Message:
        return self._connection.recv()

    def close(self) -> None:
        self._connection.close()


def open_pickled_channel() -> tuple[MessageReceiver, MessageSender]:
    reader, writer = SPAWN_CONTEXT.Pipe(duplex=False)
    return PickledReceiver(reader), PickledSender(writer)


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


def memfd_is_available() -> bool:
    if sys.platform != 'linux':
        return False
    try:
        os.close(_memfd_create('rosys-probe'))
    except (OSError, AttributeError):
        return False
    return True


class MemfdSender:
    """Frames cross as memfd file descriptors, so the receiver maps the pixels instead of copying them."""

    def __init__(self, connection: Connection) -> None:
        self._connection = connection

    def send(self, message: Message) -> None:
        if not isinstance(message, Frame):
            self._connection.send(message)
            return
        fd = _memfd_create('rosys-frame')
        try:
            os.write(fd, np.ascontiguousarray(message.array).data)
            self._connection.send(_FrameHeader(shape=message.array.shape, capture_time=message.capture_time))
            reduction.send_handle(self._connection, fd, None)
        finally:
            os.close(fd)

    def close(self) -> None:
        self._connection.close()


class MemfdReceiver:

    def __init__(self, connection: Connection) -> None:
        self._connection = connection

    def receive(self) -> Message:
        message = self._connection.recv()
        if not isinstance(message, _FrameHeader):
            return message
        fd = reduction.recv_handle(self._connection)
        try:
            buffer = mmap.mmap(fd, math.prod(message.shape), flags=mmap.MAP_SHARED | mmap.MAP_POPULATE)
        finally:
            os.close(fd)
        array = np.frombuffer(buffer, dtype=np.uint8).reshape(message.shape)  # keeps the mapping alive as its base
        return Frame(array=array, capture_time=message.capture_time)

    def close(self) -> None:
        self._connection.close()


def open_memfd_channel() -> tuple[MessageReceiver, MessageSender]:
    reader, writer = SPAWN_CONTEXT.Pipe(duplex=True)  # a socket pair, which can carry file descriptors
    return MemfdReceiver(reader), MemfdSender(writer)


open_channel = open_memfd_channel if memfd_is_available() else open_pickled_channel
"""Create the (receiver, sender) pair of the fastest channel this platform supports."""
