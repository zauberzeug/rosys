from __future__ import annotations

import asyncio
import inspect
import logging
from typing import Awaitable, Callable

from executing import Source

from .helpers import invoke

startup_coroutines: list[Awaitable] = []
tasks: list[asyncio.Task] = []
log = logging.getLogger('rosys.event')
events: list[Event] = []


class Event:

    def __init__(self) -> None:
        try:
            self.name = Source.executing(inspect.currentframe().f_back).node.parent.targets[0].id
        except:
            self.name = 'noname_event'
        self.listeners = set()
        events.append(self)

    def register(self, listener: Callable) -> Event:
        if not callable(listener):
            raise Exception('non-callable listener')
        self.listeners.add(listener)
        return self

    def unregister(self, listener: Callable) -> None:
        self.listeners.remove(listener)

    async def call(self, *args) -> None:
        '''Fires event and waits async until all registered listeners are completed'''
        for listener in self.listeners:
            try:
                await invoke(listener, *args)
            except:
                log.exception(f'could not call {listener=} for event {self.name}')

    def emit(self, *args) -> None:
        '''Fires event without waiting for the result.'''
        loop = asyncio.get_event_loop()
        for listener in self.listeners:
            try:
                result = invoke(listener, *args)
                if isinstance(result, Awaitable):
                    if loop.is_running:
                        tasks.append(loop.create_task(result, name=f'handle {self.name}'))
                    else:
                        startup_coroutines.append(result)
            except:
                log.exception(f'could not call {listener=} for event {self.name}')


def reset() -> None:
    for event in events:
        event.listeners.clear()
    for task in tasks:
        task.cancel()
    tasks.clear()
