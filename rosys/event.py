from __future__ import annotations

import asyncio
import inspect
import logging
from dataclasses import dataclass
from typing import Awaitable, Callable

from . import task_logger
from .helpers import invoke

startup_coroutines: list[Awaitable] = []
tasks: list[asyncio.Task] = []
log = logging.getLogger('rosys.event')
events: list[Event] = []


@dataclass(slots=True, kw_only=True)
class EventListener:
    callback: Callable
    filepath: str
    line: int


class Event:

    def __init__(self) -> None:
        self.listeners: list[EventListener] = list()
        events.append(self)

    def register(self, callback: Callable) -> Event:
        if not callable(callback):
            raise Exception('non-callable callback')
        if any(l.callback == callback for l in self.listeners):
            return  # NOTE: don't add duplicate listeners
        frame = inspect.currentframe().f_back
        self.listeners.append(EventListener(callback=callback, filepath=frame.f_code.co_filename, line=frame.f_lineno))
        return self

    def unregister(self, callback: Callable) -> None:
        self.listeners[:] = [l for l in self.listeners if l.callback == callback]

    async def call(self, *args) -> None:
        '''Fires event and waits async until all registered listeners are completed'''
        for listener in self.listeners:
            try:
                await invoke(listener.callback, *args)
            except:
                log.exception(f'could not call {listener=}')

    def emit(self, *args) -> None:
        '''Fires event without waiting for the result.'''
        loop = asyncio.get_event_loop()
        for listener in self.listeners:
            try:
                result = invoke(listener.callback, *args)
                if isinstance(result, Awaitable):
                    if loop.is_running:
                        name = f'{listener.filepath}:{listener.line}'
                        tasks.append(task_logger.create_task(result, name=name, loop=loop))
                    else:
                        startup_coroutines.append(result)
            except:
                log.exception(f'could not emit {listener=}')


def reset() -> None:
    for event in events:
        event.listeners.clear()
    for task in tasks:
        task.cancel()
    tasks.clear()
