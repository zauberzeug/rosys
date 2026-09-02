import abc
import asyncio
import enum
import logging
from typing import ClassVar

from nicegui import background_tasks

from .. import rosys
from .reconnect import MAX_RECONNECT_INTERVAL, clamp_reconnect_interval


class CaptureState(enum.Enum):
    """State of a self-healing capture loop."""
    CONNECTING = enum.auto()    # loop alive, opening the stream or waiting to reconnect
    STREAMING = enum.auto()     # stream open and delivering frames
    REFUSED = enum.auto()       # camera answered without a stream; loop backs off before trying again
    STOPPED = enum.auto()       # shutdown requested; loop gives up


class CaptureDevice(abc.ABC):
    """A camera device that keeps its own capture session alive.

    The loop runs one session at a time and waits `reconnect_interval` after a session that ended,
    until `shutdown()` tears it down. Subclasses supply the session in `_run_session()` and say how
    the camera is reached; everything about staying alive lives here.
    """

    REFUSED_RECONNECT_INTERVAL: ClassVar[float] = MAX_RECONNECT_INTERVAL
    """Wait between attempts while the camera answers something other than a stream.

    It is reachable and has said no, so asking again sooner cannot change the answer, and a rejected
    login or a rate limit only gets worse for being retried.
    """

    def __init__(self, *, name: str, log: logging.Logger, reconnect_interval: float = 3.0) -> None:
        self._name = name
        self.log = log
        self.reconnect_interval = reconnect_interval
        self._state = CaptureState.CONNECTING
        self._capture_task: asyncio.Task | None = None
        self._shutting_down = False

    @property
    def reconnect_interval(self) -> float:
        return self._reconnect_interval

    @reconnect_interval.setter
    def reconnect_interval(self, interval: float) -> None:
        self._reconnect_interval = clamp_reconnect_interval(interval, self.log)

    @property
    def is_connected(self) -> bool:
        """Whether the camera is delivering frames right now."""
        return self._state is CaptureState.STREAMING

    @property
    def is_active(self) -> bool:
        """Whether the self-healing capture loop is alive (streaming or waiting to reconnect)."""
        return self._capture_task is not None and not self._capture_task.done()

    @property
    def is_refused(self) -> bool:
        """Whether the camera answered the last attempt with something other than a stream."""
        return self._state is CaptureState.REFUSED

    @abc.abstractmethod
    async def _run_session(self) -> None:
        """Run one capture session until it ends."""

    async def _tear_down_session(self) -> None:  # noqa: B027
        """Release whatever the running session holds, so `shutdown()` leaves nothing behind."""

    def _retry_reason(self) -> str | None:
        """Why the next attempt is being delayed, when the device knows better than "the stream ended"."""
        return None

    def _start_capture_task(self) -> None:
        if self._shutting_down:
            self.log.warning('[%s] not starting a capture loop while the device is being torn down', self._name)
            return
        if self.is_active:
            self.log.warning('[%s] capture loop already running', self._name)
            return
        self._state = CaptureState.CONNECTING
        self._capture_task = background_tasks.create(self._run_capture_task(), name=f'capture {self._name}')

    def _keeps_running(self) -> bool:
        """Whether the calling capture task should carry on.

        A cancelled task can resume instead of ending, because the `rosys.run` helpers turn a
        cancellation into a ``None`` result; after a restart `_capture_task` is a different task.
        """
        return self._state is not CaptureState.STOPPED and self._capture_task is asyncio.current_task()

    def _set_state(self, state: CaptureState) -> None:
        """Record the state of the calling capture task, ignoring a task that has been replaced.

        Several tasks may share this attribute, so a task that no longer owns the loop must not
        report its own progress, or its end, as the state of the loop that owns it.
        """
        if self._capture_task is not asyncio.current_task():
            return
        self._state = state

    async def _run_capture_task(self) -> None:
        try:
            while self._keeps_running():
                # every attempt starts fresh: an earlier rejection says nothing about this one
                self._set_state(CaptureState.CONNECTING)
                reason = 'stream ended'
                try:
                    await self._run_session()
                except Exception as e:
                    reason = self._describe_session_error(e)
                finally:
                    if self._state is CaptureState.STREAMING:
                        self._set_state(CaptureState.CONNECTING)
                if not self._keeps_running():
                    break
                delay = self._retry_interval
                self.log.info('[%s] %s; retrying in %.1f s', self._name, self._retry_reason() or reason, delay)
                await self._wait_before_retry(delay)
        finally:
            if self._capture_task is asyncio.current_task():
                self._capture_task = None

    def _describe_session_error(self, error: Exception) -> str:
        """Turn an exception ending a session into a reason, logging a traceback only for a surprise."""
        self.log.exception('[%s] capture session failed', self._name)
        return f'capture session failed: {error}'

    @property
    def _retry_interval(self) -> float:
        """How long to wait before the next session; a refusal backs off further than a lost stream."""
        return self.REFUSED_RECONNECT_INTERVAL if self.is_refused else self.reconnect_interval

    async def _wait_before_retry(self, delay: float) -> None:
        await rosys.sleep(delay)

    def restart_capture(self) -> None:
        """Give up the running session and start a new one, e.g. after the camera moved."""
        self._stop_capture_task()
        self._start_capture_task()

    def _stop_capture_task(self) -> None:
        self._state = CaptureState.STOPPED
        if self._capture_task is not None:
            self._capture_task.cancel()
            self._capture_task = None

    async def shutdown(self) -> None:
        """Stop the capture loop and release what the running session holds."""
        self._state = CaptureState.STOPPED
        self._shutting_down = True
        try:
            await self._tear_down_session()
            await self._await_capture_task()
        finally:
            self._shutting_down = False

    async def _await_capture_task(self) -> None:
        task = self._capture_task
        if task is not None and not task.done():
            task.cancel()
            try:
                await asyncio.wait_for(asyncio.shield(task), timeout=5)
            except TimeoutError:
                self.log.warning('[%s] timeout while waiting for capture task to cancel', self._name)
            except asyncio.CancelledError:
                if not task.cancelled():
                    raise  # our own caller was cancelled, not the capture task
        if self._capture_task is task:  # a restart may have started a new loop while we waited
            self._capture_task = None
