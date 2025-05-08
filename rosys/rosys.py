import asyncio
import gc
import logging
import multiprocessing
import os
import signal
import threading
import time as pytime
import warnings
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Any, ClassVar, Literal

import numpy as np
import psutil
from nicegui import Client, app, background_tasks, ui

from . import core, event, run
from .config import Config
from .geometry.frame3d_registry import frame_registry
from .helpers import invoke, is_stopping
from .helpers import is_test as is_test_

warnings.filterwarnings('once', category=DeprecationWarning, module='rosys')

log = logging.getLogger('rosys.core')

translator: Any | None = None

is_test = is_test_()

# POSIX standard to create processes is "fork", which is inherently broken for python (see https://pythonspeed.com/articles/python-multiprocessing/)
if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')


@dataclass(slots=True, kw_only=True)
class Notification:
    time: float
    message: str


NEW_NOTIFICATION = event.Event[str]()
"""notify the user (string argument: message)"""


class _state:
    start_time: float = 0.0 if is_test else pytime.time()
    time = start_time
    last_time_request: float = start_time
    exception: BaseException | None = None  # NOTE: used for tests
    startup_finished: bool = False


def get_last_exception() -> BaseException | None:
    return _state.exception


notifications: list[Notification] = []
startup_handlers: list[Callable] = []
shutdown_handlers: list[Callable] = []
tasks: list[asyncio.Task] = []


def notify(message: str,
           type: Literal[  # pylint: disable=redefined-builtin
               'positive',
               'negative',
               'warning',
               'info',
               'ongoing',
           ] | None = None, *,
           log_level: int = logging.INFO,
           **kwargs) -> None:
    log.log(log_level, message, stacklevel=2)
    notifications.append(Notification(time=time(), message=message))
    NEW_NOTIFICATION.emit(message)
    # NOTE show notifications on all pages
    for client in Client.instances.values():
        if not client.has_socket_connection:
            continue
        with client:
            try:
                ui.notify(message, type=type, **kwargs)
            except Exception:
                log.exception('failed to call notify')


time_lock = threading.Lock()


def time() -> float:
    if is_test:
        return _state.time
    with time_lock:
        now = pytime.time()
        _state.time += (now - _state.last_time_request) * config.simulation_speed
        _state.last_time_request = now
        return _state.time


def set_time(value: float) -> None:
    assert is_test, 'only tests can change the time'
    _state.time = value


def uptime() -> float:
    return time() - _state.start_time


async def sleep(seconds: float) -> None:
    if is_test:
        sleep_end_time = time() + seconds
        while time() <= sleep_end_time:
            await asyncio.sleep(0)
    else:
        if config.simulation_speed <= 0:
            config.simulation_speed = 0.01
        scaled_seconds = seconds / config.simulation_speed
        count = int(np.ceil(scaled_seconds))
        if count > 0:
            for _ in range(count):
                await asyncio.sleep(scaled_seconds / count)
        else:
            await asyncio.sleep(0)


def _run_handler(handler: Callable) -> None:
    try:
        result = handler()
        if isinstance(result, Awaitable):
            tasks.append(background_tasks.create(result, name=handler.__qualname__))
    except Exception:
        log.exception('error while starting handler "%s"', handler.__qualname__)


class Repeater:
    tasks: ClassVar[set[asyncio.Task]] = set()

    def __init__(self, handler: Callable, interval: float) -> None:
        self.handler = handler
        self.interval = interval
        self._task: asyncio.Task | None = None

    def start(self) -> None:
        if self.running:
            return
        if _state.startup_finished:
            self._task = background_tasks.create(self._repeat())
            self.tasks.add(self._task)
        elif self.start not in startup_handlers:
            startup_handlers.append(self.start)

    async def _repeat(self) -> None:
        await sleep(self.interval)  # NOTE delaying first execution so not all actors rush in at the same time
        while True:
            start = time()
            try:
                if is_stopping():
                    log.info('%s must be stopped', self.handler)
                    break
                await invoke(self.handler)
                dt = time() - start
            except (asyncio.CancelledError, GeneratorExit):
                return
            except Exception:
                dt = time() - start
                log.exception('error in "%s"', self.handler.__qualname__)
                if self.interval == 0 and dt < 0.1:
                    delay = 0.1 - dt
                    log.warning(
                        f'"{self.handler.__qualname__}" would be called to frequently ' +
                        f'because it only took {dt*1000:.0f} ms; ' +
                        f'delaying this step for {delay*1000:.0f} ms')
                    await sleep(delay)
            try:
                await sleep(self.interval - dt)
            except (asyncio.CancelledError, GeneratorExit):
                return

    def stop(self) -> None:
        if not self._task:
            return

        if not self._task.done():
            self._task.cancel()

        self.tasks.remove(self._task)
        self._task = None

    @property
    def running(self) -> bool:
        return self._task is not None and not self._task.done()

    @staticmethod
    def stop_all() -> None:
        for repeater in Repeater.tasks:
            repeater.cancel()


def on_repeat(handler: Callable, interval: float) -> Repeater:
    repeater = Repeater(handler, interval)
    repeater.start()
    return repeater


def on_startup(handler: Callable) -> None:
    if _state.startup_finished:
        _run_handler(handler)
    else:
        startup_handlers.append(handler)


def on_shutdown(handler: Callable) -> None:
    shutdown_handlers.append(handler)


async def startup() -> None:
    if _state.startup_finished:
        raise RuntimeError('should be only executed once')
    if multiprocessing.get_start_method() != 'spawn':
        raise RuntimeError(
            'multiprocessing start method must be "spawn"; see https://pythonspeed.com/articles/python-multiprocessing/')

    _state.startup_finished = True

    for handler in startup_handlers:
        _run_handler(handler)

    for coroutine in event.startup_coroutines:
        await coroutine


async def _garbage_collection(mbyte_limit: float = 300) -> None:
    if psutil.virtual_memory().free < mbyte_limit * 1_000_000:
        log.warning('less than %s mb of memory remaining -> start garbage collection', mbyte_limit)
        gc.collect()
        await sleep(1)  # NOTE ensure all warnings appear before sending "finished" message
        log.warning('finished garbage collection')


async def _watch_emitted_events() -> None:
    try:
        for task in event.tasks:
            if task.done() and task.exception():
                _state.exception = task.exception()
                log.exception('task failed to execute', exc_info=task.exception())
        event.tasks = [t for t in event.tasks if not t.done()]
    except Exception:
        log.exception('failed to watch emitted events')


async def shutdown() -> None:
    for handler in shutdown_handlers:
        log.debug('invoking shutdown handler "%s"', handler.__qualname__)
        await invoke(handler)
    log.debug('tear down "run" tasks')
    run.tear_down()
    log.debug('stopping all repeaters')
    Repeater.stop_all()
    log.debug('canceling all remaining tasks')
    for task in tasks:
        task.cancel()
    log.debug('clearing tasks')
    tasks.clear()

    # NOTE: kill own process and all its children after uvicorn has finished (we had many restart freezes before)
    def delayed_kill():
        pytime.sleep(1)
        os.kill(os.getpid(), signal.SIGKILL)
    if not is_test:
        threading.Thread(target=delayed_kill).start()
    log.debug('finished shutdown')


def reset_before_test() -> None:
    assert is_test
    set_time(0)  # NOTE: in tests we start at zero for better readability
    _state.exception = None


def reset_after_test() -> None:
    assert is_test
    startup_handlers.clear()
    tasks.clear()
    _state.startup_finished = False
    shutdown_handlers.clear()
    event.reset()
    frame_registry.clear()

    register_base_startup_handlers()


def register_base_startup_handlers() -> None:
    on_repeat(_garbage_collection, 60)
    on_repeat(_watch_emitted_events, 0.1)


gc.disable()  # NOTE disable automatic garbage collection to optimize performance
register_base_startup_handlers()

app.on_startup(startup)
app.on_shutdown(shutdown)

core.on_repeat = on_repeat
core.on_startup = on_startup
core.on_shutdown = on_shutdown

config = Config().persistent()
