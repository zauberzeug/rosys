import asyncio
import gc
import weakref

import pytest
from nicegui import core

import rosys
from rosys.rosys import Repeater, _state, _weaken, _WeakHandler, startup_handlers
from rosys.testing import forward


def plain_function() -> str:
    return 'function'


class Ticker:
    def __init__(self, calls: list[float]) -> None:
        self.calls = calls  # external list, so the Ticker can be collected while we keep counting

    def step(self) -> None:
        self.calls.append(rosys.time())


class Handlers:
    def method(self) -> str:
        return 'method'

    @staticmethod
    def static_method() -> str:
        return 'static'

    @classmethod
    def class_method(cls) -> str:
        return 'class'


def test_weaken_returns_none_for_plain_function():
    assert _weaken(plain_function) is None


def test_weaken_returns_none_for_static_method():
    assert _weaken(Handlers().static_method) is None


def test_weaken_returns_none_for_class_method():
    assert _weaken(Handlers.class_method) is None


def test_weaken_wraps_bound_method():
    handlers = Handlers()
    handler = _weaken(handlers.method)
    assert handler is not None
    assert handler.alive  # object still there
    assert handler() == 'method'  # still forwards to the method
    assert handler.__qualname__ == 'Handlers.method'  # qualname preserved for logging


def test_weaken_does_not_keep_object_alive():
    handlers = Handlers()
    reference = weakref.ref(handlers)
    handler = _weaken(handlers.method)
    assert handler is not None

    del handlers
    gc.collect()

    assert reference() is None  # object collected despite the live handler
    assert not handler.alive  # handler reports its object is gone
    assert handler() is None  # calling it is a safe no-op


async def test_collected_object_does_not_start_orphan_task_after_startup():
    # NOTE: a weak repeater created pre-startup defers its start; if the object dies first,
    # the replayed start (during startup) must see the dead handler and not launch an orphan task.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    assert not _state.startup_finished

    handlers = Handlers()
    repeater = rosys.on_repeat(handlers.method, 0.01, weak=True)
    assert not repeater.running  # start was deferred, not launched
    assert repeater.start in startup_handlers

    del handlers
    gc.collect()
    assert isinstance(repeater.handler, _WeakHandler)
    assert not repeater.handler.alive  # handler's object is gone

    await rosys.startup()  # replays the deferred start handlers

    assert not repeater.running  # no orphan task launched

    await rosys.shutdown()
    rosys.reset_after_test()


@pytest.mark.usefixtures('rosys_integration')
async def test_repeater_can_restart_after_stop():
    calls: list[float] = []
    ticker = Ticker(calls)
    repeater = rosys.on_repeat(ticker.step, 0.1)

    await forward(0.35)
    assert repeater.running
    assert len(calls) >= 1

    repeater.stop()
    assert not repeater.running
    calls_after_stop = len(calls)
    await forward(0.5)
    assert len(calls) == calls_after_stop  # stopped: no more ticks

    repeater.start()
    assert repeater.running  # a plain stop() must not permanently disable restart
    await forward(0.35)
    assert len(calls) > calls_after_stop  # restarted: ticking again


@pytest.mark.usefixtures('rosys_integration')
async def test_weak_repeater_stops_when_object_is_collected():
    calls: list[float] = []
    ticker = Ticker(calls)
    repeater = rosys.on_repeat(ticker.step, 0.1, weak=True)
    reference = weakref.ref(ticker)

    await forward(0.35)
    assert repeater.running
    task = repeater._task  # pylint: disable=protected-access
    assert task in Repeater.tasks
    calls_while_alive = len(calls)
    assert calls_while_alive >= 1  # the repeater was actually ticking

    del ticker
    gc.collect()
    assert reference() is None  # nothing keeps the object alive

    await forward(0.5)
    assert not repeater.running  # the timer tore itself down
    assert task not in Repeater.tasks  # the done-callback pruned the registry
    assert len(calls) == calls_while_alive  # and stopped ticking after collection
