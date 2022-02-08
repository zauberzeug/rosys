import asyncio
from typing import Coroutine


class Automation:
    def __init__(self, target: Coroutine):
        self.target = target
        self._is_running = asyncio.Event()
        self._is_running.set()
        self.is_stopped = False

    @property
    def is_running(self) -> bool:
        return self._is_running.is_set()

    def __await__(self):
        target_iter = self.target.__await__()
        iter_send, iter_throw = target_iter.send, target_iter.throw
        send, message = iter_send, None
        while not self.is_stopped:
            try:
                while not self._is_running.is_set() and not self.is_stopped:
                    yield from self._is_running.wait().__await__()
            except BaseException as err:
                send, message = iter_throw, err

            if self.is_stopped:
                return

            try:
                signal = send(message)
            except StopIteration as err:
                return err.value
            else:
                send = iter_send
            try:
                message = yield signal
            except BaseException as err:
                send, message = iter_throw, err

    def pause(self):
        self._is_running.clear()

    def resume(self):
        self._is_running.set()

    def stop(self):
        self._is_running.set()
        self.is_stopped = True
