import asyncio
import logging
from collections.abc import Callable, Coroutine, Generator
from typing import Any


class Automation:
    """An automation wraps a coroutine and allows pausing and resuming it.

    Optional exception and completion handlers are called if provided.
    This class is used internally by the [automator](https://rosys.io/reference/rosys/automation/#rosys.automation.Automator).
    """

    def __init__(self,
                 coro: Coroutine,
                 exception_handler: Callable | None = None,
                 on_complete: Callable | None = None) -> None:
        self.log = logging.getLogger('rosys.automation')
        self.coro = coro
        self.exception_handler = exception_handler
        self.on_complete = on_complete
        self._can_run = asyncio.Event()
        self._can_run.set()
        self._stop = False
        self._is_waited = False

    @property
    def is_running(self) -> bool:
        return self._is_waited and self._can_run.is_set()

    @property
    def is_paused(self) -> bool:
        return self._is_waited and not self._can_run.is_set()

    @property
    def is_stopped(self) -> bool:
        return not self._is_waited

    async def run(self) -> Any | None:
        return await self

    def __await__(self) -> Generator[Any, None, Any | None]:
        try:
            self._is_waited = True
            coro_iter = self.coro.__await__()
            iter_send, iter_throw = coro_iter.send, coro_iter.throw
            send: Callable = iter_send
            message: Any = None
            while not self._stop:
                try:
                    while not self._can_run.is_set() and not self._stop:
                        yield from self._can_run.wait().__await__()  # pylint: disable=no-member
                except BaseException as err:
                    send, message = iter_throw, err

                if self._stop:
                    return None

                try:
                    signal = send(message)
                except StopIteration as err:
                    self.log.info('automation is finished')
                    if self.on_complete:
                        self.on_complete()
                    return err.value
                send = iter_send
                try:
                    message = yield signal
                except BaseException as err:
                    send, message = iter_throw, err
        except Exception as e:
            self.log.exception('automation failed')
            if self.exception_handler:
                self.exception_handler(e)
            raise
        finally:
            self._is_waited = False
        return None

    def pause(self) -> None:
        self._can_run.clear()

    def resume(self) -> None:
        self._can_run.set()

    def stop(self) -> None:
        self._can_run.set()
        self._stop = True
