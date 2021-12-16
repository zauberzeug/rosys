import asyncio
from asyncio.tasks import Task
import collections
import inspect
from aenum import Enum, auto
from typing import Awaitable, Callable, Union
import logging
import weakref

tasks: list[Task] = []
listeners = collections.defaultdict(set)
log = logging.getLogger('rosys.event')


class Id(Enum, init='value __doc__'):
    '''Event Identifier. Every event has its own set of parameters.'''

    def _generate_next_value_(name, start, count, last_values):
        '''uses enum name as value when calling auto()'''
        return name

    ROBOT_MOVED = auto(), 'triggered when a robot movement is detected'
    NEW_MACHINE_DATA = auto(), 'triggered in high frequency when machine data has been read'
    PAUSE_AUTOMATIONS = auto(), 'call this event to pause any running automations; provide a description of the cause as string parameter'
    AUTOMATIONS_STARTED = auto(), 'triggered when automations were stopped and are now running'
    NEW_NOTIFICATION = auto(), 'call this event to notify the user; provide the message as string parameter'


def register(event: Id, listener: Union[Callable, Awaitable]):
    if not callable(listener):
        raise Exception('non-callable listener')
    if listener.__name__ == '<lambda>':  # NOTE lambda functions must be stored without weakref because they will be collected otherwise
        ref = listener
    elif inspect.ismethod(listener):
        ref = weakref.WeakMethod(listener)
    else:
        ref = weakref.ref(listener)
    listeners[event].add(ref)


def unregister(event: Id, listener: Union[Callable, Awaitable]):
    listeners[event].remove(listener)


async def call(event: Id, *args):
    '''Fires event and waits async until all registered listeners are completed'''
    # if event != Id.NEW_MACHINE_DATA:
    #     log.info(f'calling {event=}')
    for listener in list(listeners.get(event, {})):
        try:
            # if event != Id.NEW_MACHINE_DATA:
            #     log.info(f'emitting {event=} with {listener=}')
            if hasattr(listener, '__name__') and listener.__name__ == '<lambda>':
                listener(*args)
                continue
            if listener() is None:
                unregister(event, listener)
                continue
            if inspect.iscoroutinefunction(listener()):
                await listener()(*args)
            else:
                listener()(*args)
        except:
            log.exception(f'could not call {listener=} for {event=}')


def emit(event: Id, *args):
    '''Fires event without waiting for the result.'''

    # if event != Id.ROBOT_MOVED:
    #     log.info(f'emitting {event=}')
    loop = asyncio.get_event_loop()
    for listener in list(listeners.get(event, {})):
        # if event != Id.ROBOT_MOVED:
        #     log.info(f'emitting {event=} with {listener=}')
        try:
            if hasattr(listener, '__name__') and listener.__name__ == '<lambda>':
                tasks.append(loop.run_in_executor(None, listener, *args))
                continue
            if listener() is None:
                unregister(event, listener)
                continue
            if inspect.iscoroutinefunction(listener()):
                tasks.append(loop.create_task(listener()(*args), name=f'handle {event=}'))
            else:
                tasks.append(loop.run_in_executor(None, listener(),  *args))
        except:
            log.exception(f'could not call {listener=} for {event=}')
