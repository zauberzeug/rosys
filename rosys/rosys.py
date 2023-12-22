import asyncio
import gc
import logging
import multiprocessing
import os
import signal
import threading
import time as pytime
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Literal, Optional

import numpy as np
import psutil
from nicegui import Client, app, background_tasks, ui

from . import event, persistence, run
from .config import Config
from .helpers import invoke, is_stopping
from .helpers import is_test as is_test_

log = logging.getLogger('rosys.core')

config = Config()
translator: Optional[Any] = None

is_test = is_test_()

# POSIX standard to create processes is "fork", which is inherently broken for python (see https://pythonspeed.com/articles/python-multiprocessing/)
if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')


@dataclass(slots=True, kw_only=True)
class Notification:
    time: float
    message: str


NEW_NOTIFICATION = event.Event()
"""notify the user (string argument: message)"""


class _state:
    start_time: float = 0.0 if is_test else pytime.time()
    time = start_time
    last_time_request: float = start_time
    exception: Optional[BaseException] = None  # NOTE: used for tests


def get_last_exception() -> Optional[BaseException]:
    return _state.exception


notifications: list[Notification] = []
repeat_handlers: list[tuple[Callable, float]] = []
startup_handlers: list[Callable] = []
shutdown_handlers: list[Callable] = []
tasks: list[asyncio.Task] = []


def notify(message: str,
           type: Optional[Literal[  # pylint: disable=redefined-builtin
               'positive',
               'negative',
               'warning',
               'info',
               'ongoing',
           ]] = None) -> None:
    log.info(message)
    notifications.append(Notification(time=time(), message=message))
    NEW_NOTIFICATION.emit(message)
    # NOTE show notifications on all pages
    for client in Client.instances.values():
        if not client.has_socket_connection:
            continue
        with client:
            try:
                ui.notify(message, type=type)
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
        log.exception(f'error while starting handler "{handler.__qualname__}"')


def _start_loop(handler: Callable, interval: float) -> None:
    log.debug(f'starting loop "{handler.__qualname__}" with interval {interval:.3f}s')
    tasks.append(background_tasks.create(_repeat_one_handler(handler, interval), name=handler.__qualname__))


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
                _state.exception = task.exception()
                log.exception('task failed to execute', exc_info=task.exception())
        event.tasks = [t for t in event.tasks if not t.done()]
    except Exception:
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
        except Exception:
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
    repeat_handlers[3:] = []  # NOTE: remove all but internal handlers
    shutdown_handlers.clear()
    event.reset()


gc.disable()  # NOTE disable automatic garbage collection to optimize performance
on_repeat(_garbage_collection, 10 * 60)
on_repeat(_watch_emitted_events, 0.1)
on_repeat(persistence.backup, 10)

app.on_startup(startup)
app.on_shutdown(shutdown)
