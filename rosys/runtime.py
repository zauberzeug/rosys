import asyncio
import gc
import logging
import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
import psutil

from . import event, persistence, run
from .helpers import invoke, is_test
from .task_logger import create_task


@dataclass(slots=True, kw_only=True)
class Notification:
    time: float
    message: str


class Runtime:
    NEW_NOTIFICATION = event.Event()
    '''notify the user (string argument: message)'''

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.runtime')

        self.is_test = is_test()
        self._time = time.time()
        self._start_time = self.time
        self.notifications: list[Notification] = []
        self._exception: Optional[Exception] = None  # NOTE: used for tests

        self.repeat_handlers: list[tuple[Callable, float]] = []
        self.startup_handlers: list[Callable] = []
        self.shutdown_handlers: list[Callable] = []
        self.tasks: list[asyncio.Task] = []

        gc.disable()  # NOTE disable automatic garbage collection to optimize performance
        self.on_repeat(self._garbage_collection, 10 * 60)
        self.on_repeat(self._watch_emitted_events, 0.1)
        self.on_repeat(persistence.backup, 10)

    def notify(self, message: str) -> None:
        self.log.info(message)
        self.notifications.append(Notification(time=self.time, message=message))
        self.NEW_NOTIFICATION.emit(message)

    @property
    def time(self) -> float:
        return self._time if self.is_test else time.time()

    def set_time(self, value: float) -> None:
        assert self.is_test, 'only tests can change the time'
        self._time = value

    @property
    def uptime(self) -> float:
        return self.time - self._start_time

    async def sleep(self, seconds: float) -> None:
        if self.is_test:
            sleep_end_time = self.time + seconds
            while self.time <= sleep_end_time:
                await asyncio.sleep(0)
        else:
            count = int(np.ceil(seconds))
            if count > 0:
                for _ in range(count):
                    await asyncio.sleep(seconds / count)
            else:
                await asyncio.sleep(0)

    def on_repeat(self, handler: Callable, interval: float) -> None:
        self.repeat_handlers.append((handler, interval))

    def on_startup(self, handler: Callable) -> None:
        self.startup_handlers.append(handler)

    def on_shutdown(self, handler: Callable) -> None:
        self.shutdown_handlers.append(handler)

    async def startup(self) -> None:
        if self.tasks:
            raise Exception('should be only executed once')

        persistence.restore()

        for handler in self.startup_handlers:
            try:
                await invoke(handler)
            except:
                self.log.exception(f'error while starting handler "{handler.__qualname__}"')
                continue

        for coroutine in event.startup_coroutines:
            await coroutine

        for handler, interval in self.repeat_handlers:
            self.log.debug(f'starting loop "{handler.__qualname__}" with interval {interval:.3f}s')
            self.tasks.append(create_task(self._repeat_one_handler(handler, interval), name=handler.__qualname__))

    async def _garbage_collection(self, mbyte_limit: float = 300) -> None:
        if psutil.virtual_memory().free < mbyte_limit * 1_000_000:
            self.log.warning(f'less than {mbyte_limit} mb of memory remaining -> start garbage collection')
            gc.collect()
            await runtime.sleep(1)  # NOTE ensure all warnings appear before sending "finished" message
            self.log.warning('finished garbage collection')

    async def _watch_emitted_events(self) -> None:
        try:
            for task in event.tasks:
                if task.done() and task.exception():
                    self._exception = task.exception()
                    self.log.exception('task failed to execute', exc_info=task.exception())
            event.tasks = [t for t in event.tasks if not t.done()]
        except:
            self.log.exception('failed to watch emitted events')

    async def _repeat_one_handler(self, handler: Callable, interval: float) -> None:
        await self.sleep(interval)  # NOTE delaying first execution so not all actors rush in at the same time
        while True:
            start = self.time
            try:
                await invoke(handler)
                dt = self.time - start
            except (asyncio.CancelledError, GeneratorExit):
                return
            except:
                dt = self.time - start
                self.log.exception(f'error in "{handler.__qualname__}"')
                if interval == 0 and dt < 0.1:
                    delay = 0.1 - dt
                    self.log.warning(
                        f'"{handler.__qualname__}" would be called to frequently ' +
                        f'because it only took {dt*1000:.0f} ms; ' +
                        f'delaying this step for {delay*1000:.0f} ms')
                    await self.sleep(delay)
            try:
                await self.sleep(interval - dt)
            except (asyncio.CancelledError, GeneratorExit):
                return

    async def shutdown(self) -> None:
        persistence.backup(force=True)
        run.tear_down()
        [t.cancel() for t in self.tasks]
        self.tasks.clear()
        for handler in self.shutdown_handlers:
            await invoke(handler)

    def reset_before_test(self) -> None:
        assert is_test()
        self.set_time(0)  # NOTE: in tests we start at zero for better readability
        self._exception = None

    def reset_after_test(self) -> None:
        assert is_test()
        runtime.startup_handlers.clear()
        runtime.repeat_handlers[3:] = []  # NOTE: remove all but internal handlers
        runtime.shutdown_handlers.clear()
        event.reset()


runtime = Runtime()
