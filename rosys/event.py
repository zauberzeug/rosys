import asyncio
import collections
import inspect
from enum import Enum, auto
from typing import Awaitable, Callable, Union
import logging

listeners = collections.defaultdict(set)
log = logging.getLogger('rosys.event')


class Id(Enum):
    def _generate_next_value_(name, start, count, last_values):
        return name

    NEW_MACHINE_DATA = auto()
    PAUSE_AUTOMATIONS = auto()


def register(event: id, listener: Union[Callable, Awaitable]):
    listeners[event].add(listener)


def unregister(event: id, listener: Union[Callable, Awaitable]):
    listeners[event].remove(listener)


async def call(event: id, data=None):
    for listener in listeners.get(event, {}):
        try:
            if inspect.iscoroutinefunction(listener):
                await listener(data)
            else:
                listener(data)
        except:
            log.exception(f'could not execute {listener=} for {event=}')
