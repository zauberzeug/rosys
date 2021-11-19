import asyncio
import collections
import inspect
from aenum import Enum, auto
from typing import Awaitable, Callable, Union
import logging

listeners = collections.defaultdict(set)
log = logging.getLogger('rosys.event')


class Id(Enum):
    '''Event Identifier. Every event has its own set of parameters.'''
    def _generate_next_value_(name, start, count, last_values):
        return name

    NEW_MACHINE_DATA = auto(), 'triggered in high frequency whenever machine data had been read; provides world object where the data has been written to.'
    PAUSE_AUTOMATIONS = auto(), 'call this event to pause any running automations; provide a description of the cause as string parameter.'


def register(event: Id, listener: Union[Callable, Awaitable]):
    listeners[event].add(listener)


def unregister(event: Id, listener: Union[Callable, Awaitable]):
    listeners[event].remove(listener)


async def call(event: Id, *args):
    for listener in listeners.get(event, {}):
        try:
            if inspect.iscoroutinefunction(listener):
                await listener(*args)
            else:
                listener(*args)
        except:
            log.exception(f'could not execute {listener=} for {event=}')
