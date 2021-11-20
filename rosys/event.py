import asyncio
import collections
import inspect
from aenum import Enum, auto
from typing import Awaitable, Callable, Union
import logging
import weakref

listeners = collections.defaultdict(set)
log = logging.getLogger('rosys.event')


class Id(Enum):
    '''Event Identifier. Every event has its own set of parameters.'''
    def _generate_next_value_(name, start, count, last_values):
        return name

    NEW_MACHINE_DATA = auto(), 'triggered in high frequency whenever machine data had been read; provides world object where the data has been written to.'
    PAUSE_AUTOMATIONS = auto(), 'call this event to pause any running automations; provide a description of the cause as string parameter.'


def register(event: Id, listener: Union[Callable, Awaitable]):
    ref = weakref.WeakMethod(listener) if hasattr(listener, '__self__') else weakref.ref(listener)
    listeners[event].add(ref)


def unregister(event: Id, listener: Union[Callable, Awaitable]):
    listeners[event].remove(listener)


async def call(event: Id, *args):
    for listener in list(listeners.get(event, {})):
        try:
            if listener() is None:
                unregister(event, listener)
                continue
            if inspect.iscoroutinefunction(listener()):
                await listener()(*args)
            else:
                listener()(*args)
        except:
            log.exception(f'could not execute {listener=} for {event=}')
