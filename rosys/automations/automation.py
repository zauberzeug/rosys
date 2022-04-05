import asyncio
import logging
from typing import Callable, Coroutine, Optional


class Automation:
    def __init__(self,
                 coro: Coroutine,
                 exception_handler: Optional[Callable] = None,
                 on_complete: Optional[Callable] = None):
        self.log = logging.getLogger('automations.automation')
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

    def __await__(self):
        try:
            self._is_waited = True
            coro_iter = self.coro.__await__()
            iter_send, iter_throw = coro_iter.send, coro_iter.throw
            send, message = iter_send, None
            while not self._stop:
                try:
                    while not self._can_run.is_set() and not self._stop:
                        yield from self._can_run.wait().__await__()
                except BaseException as err:
                    send, message = iter_throw, err

                if self._stop:
                    return

                try:
                    signal = send(message)
                except StopIteration as err:
                    self.log.info('automation is finished')
                    if self.on_complete:
                        self.on_complete()
                    return err.value
                else:
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

    def pause(self):
        self._can_run.clear()

    def resume(self):
        self._can_run.set()

    def stop(self):
        self._can_run.set()
        self._stop = True
