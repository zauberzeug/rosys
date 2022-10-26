import asyncio
import gc
import logging
import multiprocessing
import os
import signal
import sys
import threading
import time as pytime
from dataclasses import dataclass
from typing import Awaitable, Callable, Optional

import numpy as np
import psutil
from nicegui import globals as nicegui_globals
from nicegui import ui
from nicegui.page import Page

from . import event, persistence, run
from .config import Config
from .helpers import invoke
from .task_logger import create_task

log = logging.getLogger('rosys.core')

config = Config()

is_test = 'pytest' in sys.modules

# POSIX standard to create processes is "fork", which is inherently broken for python (see https://pythonspeed.com/articles/python-multiprocessing/)
if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')


@dataclass(slots=True, kw_only=True)
class Notification:
    time: float
    message: str


NEW_NOTIFICATION = event.Event()
'''notify the user (string argument: message)'''


_start_time: float = 0.0 if is_test else pytime.time()
_time = _start_time
_last_time_request: float = _start_time
speed: float = 1.0

notifications: list[Notification] = []
_exception: Optional[Exception] = None  # NOTE: used for tests
repeat_handlers: list[tuple[Callable, float]] = []
startup_handlers: list[Callable] = []
shutdown_handlers: list[Callable] = []
tasks: list[asyncio.Task] = []


def notify(message: str) -> None:
    log.info(message)
    notifications.append(Notification(time=time, message=message))
    NEW_NOTIFICATION.emit(message)
    # NOTE show notifications on all pages
    for page in Page.instances.values():
        assert isinstance(page, Page)
        with nicegui_globals.within_view(page.view):
            try:
                ui.notify(message)
            except:
                log.exception('failed to call notify')


time_lock = threading.Lock()


def time() -> float:
    global _time, _last_time_request
    if is_test:
        return _time
    with time_lock:
        now = pytime.time()
        _time += (now - _last_time_request) * speed
        _last_time_request = now
        return _time


def set_time(value: float) -> None:
    assert is_test, 'only tests can change the time'
    global _time
    _time = value


def uptime() -> float:
    return time() - _start_time


async def sleep(seconds: float) -> None:
    if is_test:
        sleep_end_time = time() + seconds
        while time() <= sleep_end_time:
            await asyncio.sleep(0)
    else:
        count = int(np.ceil(seconds))
        if count > 0:
            for _ in range(count):
                await asyncio.sleep(seconds / count)
        else:
            await asyncio.sleep(0)


def _run_handler(handler: Callable) -> None:
    try:
        result = handler()
        if isinstance(result, Awaitable):
            tasks.append(create_task(result, name=handler.__qualname__))
    except:
        log.exception(f'error while starting handler "{handler.__qualname__}"')


def _start_loop(handler: Callable, interval: float) -> None:
    log.debug(f'starting loop "{handler.__qualname__}" with interval {interval:.3f}s')
    tasks.append(create_task(_repeat_one_handler(handler, interval), name=handler.__qualname__))


def on_repeat(handler: Callable, interval: float) -> None:
    if tasks:  # RoSys is already running
        _start_loop(handler, interval)
    else:
        repeat_handlers.append((handler, interval))


def on_startup(handler: Callable) -> None:
    if tasks:  # RoSys is already running
        _run_handler(handler)
    else:
        startup_handlers.append(handler)


def on_shutdown(handler: Callable) -> None:
    shutdown_handlers.append(handler)


async def startup() -> None:
    if tasks:
        raise RuntimeError('should be only executed once')
    if multiprocessing.get_start_method() != 'spawn':
        raise RuntimeError(
            'multiprocessing start method must be "spawn"; see https://pythonspeed.com/articles/python-multiprocessing/')

    persistence.restore()

    for handler in startup_handlers:
        _run_handler(handler)

    for coroutine in event.startup_coroutines:
        await coroutine

    for handler, interval in repeat_handlers:
        _start_loop(handler, interval)


def is_stopping() -> bool:
    return nicegui_globals.state == nicegui_globals.State.STOPPING


async def _garbage_collection(mbyte_limit: float = 300) -> None:
    if psutil.virtual_memory().free < mbyte_limit * 1_000_000:
        log.warning(f'less than {mbyte_limit} mb of memory remaining -> start garbage collection')
        gc.collect()
        await sleep(1)  # NOTE ensure all warnings appear before sending "finished" message
        log.warning('finished garbage collection')


async def _watch_emitted_events() -> None:
    try:
        for task in event.tasks:
            if task.done() and task.exception():
                global _exception
                _exception = task.exception()
                log.exception('task failed to execute', exc_info=task.exception())
        event.tasks = [t for t in event.tasks if not t.done()]
    except:
        log.exception('failed to watch emitted events')


async def _repeat_one_handler(handler: Callable, interval: float) -> None:
    await sleep(interval)  # NOTE delaying first execution so not all actors rush in at the same time
    while True:
        start = time()
        try:
            if is_stopping():
                log.info(f'{handler} must be stopped')
                break
            await invoke(handler)
            dt = time() - start
        except (asyncio.CancelledError, GeneratorExit):
            return
        except:
            dt = time() - start
            log.exception(f'error in "{handler.__qualname__}"')
            if interval == 0 and dt < 0.1:
                delay = 0.1 - dt
                log.warning(
                    f'"{handler.__qualname__}" would be called to frequently ' +
                    f'because it only took {dt*1000:.0f} ms; ' +
                    f'delaying this step for {delay*1000:.0f} ms')
                await sleep(delay)
        try:
            await sleep(interval - dt)
        except (asyncio.CancelledError, GeneratorExit):
            return


async def shutdown() -> None:
    for handler in shutdown_handlers:
        log.debug(f'invoking shutdown handler "{handler.__qualname__}"')
        await invoke(handler)
    log.debug('creating data backup')
    await persistence.backup(force=True)
    log.debug('tear down "run" tasks')
    run.tear_down()
    log.debug('canceling all remaining tasks')
    [t.cancel() for t in tasks]
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
    global _exception
    _exception = None


def reset_after_test() -> None:
    assert is_test
    startup_handlers.clear()
    repeat_handlers[3:] = []  # NOTE: remove all but internal handlers
    shutdown_handlers.clear()
    event.reset()


gc.disable()  # NOTE disable automatic garbage collection to optimize performance
on_repeat(_garbage_collection, 10 * 60)
on_repeat(_watch_emitted_events, 0.1)
on_repeat(persistence.backup, 10)

ui.on_startup(startup)
ui.on_shutdown(shutdown)
