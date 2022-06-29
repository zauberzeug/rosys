import logging
from asyncio import CancelledError, Task
from typing import Awaitable, Callable

from . import run
from .core import core
from .task_logger import create_task

repeat_handlers: list[tuple[Callable | Awaitable, float]] = []
startup_handlers: list[Callable | Awaitable] = []
shutdown_handlers: list[Callable | Awaitable] = []
tasks: list[Task] = []
log = logging.getLogger('rosys.lifecycle')


def on_repeat(handler: Callable | Awaitable, interval: float) -> None:
    repeat_handlers.append((handler, interval))


def on_startup(handler: Callable | Awaitable) -> None:
    startup_handlers.append(handler)


def on_shutdown(handler: Callable | Awaitable) -> None:
    shutdown_handlers.append(handler)


async def startup() -> None:
    if tasks:
        raise Exception('should be only executed once')

    for handler in startup_handlers:
        try:
            await handler()
        except:
            log.exception(f'error while starting handler "{handler.__qualname__}"')
            continue

    for handler, interval in repeat_handlers:
        log.debug(f'starting loop "{handler.__qualname__}" with interval {interval:.3f}s')
        tasks.append(create_task(_repeat_one_handler(handler, interval), name=handler.__qualname__))


async def _repeat_one_handler(handler: Callable | Awaitable, interval: float) -> None:
    await core.sleep(interval)  # NOTE delaying first execution so not all actors rush in at the same time
    while True:
        start = core.time
        try:
            await handler()
            dt = core.time - start
        except (CancelledError, GeneratorExit):
            return
        except:
            dt = core.time - start
            log.exception(f'error in "{handler.__qualname__}"')
            if interval == 0 and dt < 0.1:
                delay = 0.1 - dt
                log.warning(
                    f'"{handler.__qualname__}" would be called to frequently ' +
                    f'because it only took {dt*1000:.0f} ms; ' +
                    f'delaying this step for {delay*1000:.0f} ms')
                await core.sleep(delay)
        try:
            await core.sleep(interval - dt)
        except (CancelledError, GeneratorExit):
            return


async def shutdown() -> None:
    run.tear_down()
    [t.cancel() for t in tasks]
    for handler in shutdown_handlers:
        await handler()
