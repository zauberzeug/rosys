from __future__ import annotations

import asyncio
import functools
import inspect
import logging
import weakref
from typing import Callable

from executing import Source

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
        if listener.__name__ == '<lambda>':  # NOTE lambda functions must be stored without weakref because they will be collected otherwise
            ref = listener
        elif inspect.ismethod(listener):
            ref = weakref.WeakMethod(listener)
        else:
            ref = weakref.ref(listener)
        self.listeners.add(ref)
        return self

    def unregister(self, listener: Callable) -> None:
        marked = []
        for registered in self.listeners:
            if hasattr(listener, '__name__') and listener.__name__ == '<lambda>' and listener == registered:
                marked.append(listener)
            if registered() == listener:
                marked.append(registered)
        [self.listeners.remove(m) for m in marked]

    async def call(self, *args) -> None:
        '''Fires event and waits async until all registered listeners are completed'''
        for listener in list(self.listeners):
            try:
                if hasattr(listener, '__name__') and listener.__name__ == '<lambda>':
                    listener(*args)
                    continue
                callback = listener()
                if callback is None:
                    self.listeners.remove(listener)
                    continue
                if iscoroutinefunction(listener):
                    await callback(*args)
                else:
                    callback(*args)
            except:
                log.exception(f'could not call {listener=} for event {self.name}')

    def emit(self, *args) -> None:
        '''Fires event without waiting for the result.'''
        assert asyncio.get_running_loop()

        loop = asyncio.get_event_loop()
        for listener in list(self.listeners):
            try:
                if hasattr(listener, '__name__') and listener.__name__ == '<lambda>':
                    listener(*args)
                    continue
                callback = listener()
                if callback is None:
                    self.listeners.remove(listener)
                    continue
                if iscoroutinefunction(listener):
                    tasks.append(loop.create_task(callback(*args), name=f'handle {self.name}'))
                else:
                    callback(*args)
            except:
                log.exception(f'could not call {listener=} for event {self.name}')


@functools.lru_cache(maxsize=100)
def iscoroutinefunction(listener):
    return inspect.iscoroutinefunction(listener())


def reset() -> None:
    for event in events:
        event.listeners.clear()
    for task in tasks:
        task.cancel()
    tasks.clear()
