import asyncio
import gc
import inspect
import logging
import os
import signal
import threading
import time as pytime
import warnings
import weakref
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Any, ClassVar, Literal

import numpy as np
import psutil
from nicegui import Client, Event, app, background_tasks, ui

from . import core, helpers, run
from .config import Config
from .geometry.frame3d_registry import frame_registry
from .helpers import invoke, is_stopping
from .persistence import Persistable

warnings.filterwarnings('once', category=DeprecationWarning, module='rosys')

log = logging.getLogger('rosys.core')

translator: Any | None = None

core.is_test = is_test = helpers.is_test()


@dataclass(slots=True, kw_only=True)
class Notification:
    time: float
    message: str


NEW_NOTIFICATION = Event[str]()
"""notify the user (string argument: message)"""


class _state:
    start_time: float = 0.0 if is_test else pytime.time()
    time = start_time
    last_time_request: float = start_time
    exception: BaseException | None = None  # NOTE: used for tests
    startup_finished: bool = False
    is_simulation: bool = False


def is_simulation() -> bool:
    """Returns whether system time is being simulated."""
    return _state.is_simulation


def enter_simulation() -> None:
    """Enter system time simulation mode."""
    _state.is_simulation = True


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
        if _state.is_simulation:
            _state.time += (now - _state.last_time_request) * config.simulation_speed
        else:
            _state.time = now
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


def _handler_name(handler: Callable) -> str:
    return getattr(handler, '__qualname__', repr(handler))  # partials and callable instances lack __qualname__


def _run_handler(handler: Callable) -> None:
    try:
        result = handler()
        if isinstance(result, Awaitable):
            tasks.append(background_tasks.create(result, name=_handler_name(handler)))
    except Exception:
        log.exception('error while starting handler "%s"', _handler_name(handler))


class _WeakHandler:
    def __init__(self, method: Callable) -> None:
        self._weak = weakref.WeakMethod(method)
        self.__qualname__ = method.__qualname__

    @property
    def alive(self) -> bool:
        return self._weak() is not None

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        method = self._weak()
        return None if method is None else method(*args, **kwargs)


def _prepare_handler(handler: Callable) -> Callable:
    """Weaken bound methods so the repetition ends with their object; keep plain functions strong.

    Lambdas, capturing closures, partials and other callables are rejected
    because no single lifetime behavior can serve their intent.
    """
    if inspect.ismethod(handler):
        try:
            return _WeakHandler(handler)
        except TypeError as e:
            raise TypeError(f'the object behind "{handler.__qualname__}" does not support weak references; '
                            'add "__weakref__" to its __slots__ or use "weakref_slot=True" for dataclasses') from e
    if inspect.isfunction(handler):
        if handler.__name__ == '<lambda>':
            raise TypeError('lambdas are not supported; '
                            'pass a bound method (bind arguments as attributes or default parameters) '
                            'or a plain function to repeat until shutdown')
        if handler.__closure__:
            raise TypeError(f'"{handler.__qualname__}" captures variables; '
                            'turn them into attributes of a bound method '
                            'or default parameters of a plain function')
        return handler
    raise TypeError(f'unsupported handler {_handler_name(handler)}; '
                    'pass a bound method (repeats until its object dies) '
                    'or a plain function (repeats until shutdown)')


class Repeater:
    tasks: ClassVar[set[asyncio.Task]] = set()

    def __init__(self, handler: Callable, interval: float) -> None:
        self.interval = interval
        self._task: asyncio.Task | None = None
        self.handler = _prepare_handler(handler)

    @property
    def _alive(self) -> bool:
        return not isinstance(self.handler, _WeakHandler) or self.handler.alive  # strong handlers are always alive

    def start(self) -> None:
        if self.running or not self._alive:
            return
        if _state.startup_finished:
            self._task = background_tasks.create(self._repeat())
            self.tasks.add(self._task)
            self._task.add_done_callback(self._handle_task_done)
        elif self.start not in startup_handlers:
            startup_handlers.append(self.start)

    def _handle_task_done(self, task: asyncio.Task) -> None:
        self.tasks.discard(task)
        if self._task is task:  # not a stale callback from a task replaced by a restart
            self._task = None

    async def _repeat(self) -> None:
        await sleep(self.interval)  # NOTE delaying first execution so not all actors rush in at the same time
        while self._alive:  # a weak handler whose object was collected ends the loop
            start = time()
            try:
                if is_stopping():
                    log.info('%s must be stopped', _handler_name(self.handler))
                    break
                await invoke(self.handler)
                dt = time() - start
            except (asyncio.CancelledError, GeneratorExit):
                return
            except Exception:
                dt = time() - start
                log.exception('error in "%s"', _handler_name(self.handler))
                if self.interval == 0 and dt < 0.1:
                    delay = 0.1 - dt
                    log.warning(
                        f'"{_handler_name(self.handler)}" would be called to frequently ' +
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

        self.tasks.discard(self._task)
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

    _state.startup_finished = True

    for handler in startup_handlers:
        _run_handler(handler)


async def _garbage_collection() -> None:
    if psutil.virtual_memory().free < config.garbage_collection_mbyte_limit * 1_000_000:
        log.warning('less than %s mb of memory remaining -> start garbage collection',
                    config.garbage_collection_mbyte_limit)
        gc.collect()
        await sleep(1)  # NOTE ensure all warnings appear before sending "finished" message
        log.warning('finished garbage collection')


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
    frame_registry.clear()

    Persistable.instances.clear()
    register_base_startup_handlers()


def register_base_startup_handlers() -> None:
    on_repeat(_garbage_collection, 60)


gc.disable()  # NOTE disable automatic garbage collection to optimize performance
register_base_startup_handlers()

app.on_startup(startup)
app.on_shutdown(shutdown)

core.on_repeat = on_repeat
core.on_startup = on_startup
core.on_shutdown = on_shutdown
core.sleep = sleep
core.time = time

config = Config().persistent()
