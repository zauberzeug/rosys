import asyncio
import gc
import logging
import weakref

import pytest
from nicegui import core

import rosys
from rosys.rosys import Repeater, _state, _weaken, startup_handlers
from rosys.testing import forward


def _on_dead() -> None:
    pass


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
    assert _weaken(plain_function, _on_dead) is None


def test_weaken_returns_none_for_static_method():
    assert _weaken(Handlers().static_method, _on_dead) is None


def test_weaken_returns_none_for_class_method():
    assert _weaken(Handlers.class_method, _on_dead) is None


def test_repeater_warns_when_weak_handler_cannot_be_weakened(caplog: pytest.LogCaptureFixture):
    with caplog.at_level(logging.WARNING):
        repeater = Repeater(plain_function, 0.1, weak=True)
    assert repeater.handler is plain_function  # falls back to the original handler
    assert 'weak=True has no effect' in caplog.text


def test_weaken_wraps_bound_method():
    handlers = Handlers()
    wrapper = _weaken(handlers.method, _on_dead)
    assert getattr(wrapper, '__self__', None) is None  # wrapped, not the bound method
    assert wrapper() == 'method'  # still forwards to the method
    assert wrapper.__qualname__ == 'Handlers.method'  # qualname preserved for logging


def test_weaken_does_not_keep_object_alive():
    dead = []
    handlers = Handlers()
    reference = weakref.ref(handlers)
    wrapper = _weaken(handlers.method, lambda: dead.append(True))

    del handlers
    gc.collect()

    assert reference() is None  # object collected despite the live wrapper
    assert dead == [True]  # on_dead fired on collection
    assert wrapper() is None  # wrapper became a no-op


async def test_dead_flag_prevents_orphan_task_when_object_dies_before_startup():
    # NOTE: a weak repeater created pre-startup defers its start; if the object dies first,
    # the finalizer must keep the replayed start (during startup) from launching an orphan task.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    assert not _state.startup_finished

    handlers = Handlers()
    repeater = rosys.on_repeat(handlers.method, 0.01, weak=True)
    assert not repeater.running  # start was deferred, not launched
    assert repeater.start in startup_handlers

    del handlers
    gc.collect()
    assert repeater._dead  # finalizer marked it dead before startup  # pylint: disable=protected-access

    await rosys.startup()  # replays the deferred start handlers

    assert not repeater.running  # guard prevented the orphan task

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
    calls_while_alive = len(calls)
    assert calls_while_alive >= 1  # the repeater was actually ticking

    del ticker
    gc.collect()
    assert reference() is None  # nothing keeps the object alive

    await forward(0.5)
    assert not repeater.running  # the timer tore itself down
    assert len(calls) == calls_while_alive  # and stopped ticking after collection
