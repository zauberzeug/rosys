from __future__ import annotations

import asyncio
import functools
import inspect
import logging
import weakref
from asyncio.tasks import Task
from typing import Callable, Optional

tasks: list[Task] = []
log = logging.getLogger('rosys.event')


class Event:

    def __init__(self, description: Optional[str] = None) -> None:
        self.description = description
        self.listeners = set()

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

    def unregistered(self, listener: Callable) -> None:
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
                log.exception(f'could not call {listener=} for event {self}')

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
                    self.unregistered(listener)
                    continue
                if iscoroutinefunction(listener):
                    tasks.append(loop.create_task(callback(*args), name=f'handle {self}'))
                else:
                    callback(*args)
            except:
                log.exception(f'could not call {listener=} for event {self}')


@functools.lru_cache(maxsize=100)
def iscoroutinefunction(listener):
    return inspect.iscoroutinefunction(listener())
