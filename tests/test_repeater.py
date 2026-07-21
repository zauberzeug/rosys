import asyncio
import functools
import gc
import logging
import weakref
from dataclasses import dataclass

import pytest
from nicegui import core

import rosys
from rosys.rosys import Repeater, _handler_name, _prepare_handler, _state, _WeakHandler, startup_handlers
from rosys.testing import forward


@pytest.fixture
def rosys_log(rosys_integration: None, caplog: pytest.LogCaptureFixture):
    logger = logging.getLogger('rosys.core')  # NOTE: the test log configuration disables propagation to the root
    logger.addHandler(caplog.handler)
    yield caplog
    logger.removeHandler(caplog.handler)


def plain_function() -> str:
    return 'function'


def _transparent_decorator(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        return func(*args, **kwargs)
    return wrapper


@_transparent_decorator
def decorated_function() -> str:
    return 'decorated'


class Ticker:
    def __init__(self, calls: list[float]) -> None:
        self.calls = calls  # external list, so the Ticker can be collected while we keep counting

    def step(self) -> None:
        self.calls.append(rosys.time())


class CallableHandler:
    def __call__(self) -> None:
        pass


@dataclass(slots=True, kw_only=True)
class SlotsValue:  # __slots__ without __weakref__ -> not weak-referenceable
    value: int = 0

    def method(self) -> None:
        pass


class Handlers:
    def method(self) -> str:
        return 'method'

    @staticmethod
    def static_method() -> str:
        return 'static'

    @classmethod
    def class_method(cls) -> str:
        return 'class'


def test_plain_function_stays_strong():
    assert _prepare_handler(plain_function) is plain_function


def test_static_method_stays_strong():
    assert _prepare_handler(Handlers().static_method) is Handlers.static_method


def test_capture_free_nested_function_stays_strong():
    def nested() -> None:
        pass
    assert _prepare_handler(nested) is nested


def test_decorated_plain_function_stays_strong():
    assert _prepare_handler(decorated_function) is decorated_function


def test_decorated_bound_method_is_rejected():
    wrapped_method = _transparent_decorator(Handlers().method)  # the closure captures the method strongly
    with pytest.raises(TypeError, match='captures variables'):
        _prepare_handler(wrapped_method)


def test_bound_method_becomes_weak():
    handlers = Handlers()
    handler = _prepare_handler(handlers.method)
    assert isinstance(handler, _WeakHandler)
    assert handler.alive  # object still there
    assert handler() == 'method'  # still forwards to the method
    assert handler.__qualname__ == 'Handlers.method'  # qualname preserved for logging


def test_class_method_becomes_weak_on_the_everlasting_class():
    handler = _prepare_handler(Handlers.class_method)
    assert isinstance(handler, _WeakHandler)
    assert handler.alive
    assert handler() == 'class'


def test_bound_dunder_call_becomes_weak():
    callable_handler = CallableHandler()
    handler = _prepare_handler(callable_handler.__call__)
    assert isinstance(handler, _WeakHandler)
    assert handler.alive


def test_lambda_is_rejected():
    with pytest.raises(TypeError, match='lambdas are not supported'):
        Repeater(lambda: None, 0.1)


def test_capturing_closure_is_rejected():
    x = 42

    def closure() -> int:
        return x
    with pytest.raises(TypeError, match='captures variables'):
        Repeater(closure, 0.1)


def test_partial_is_rejected():
    with pytest.raises(TypeError, match='unsupported handler'):
        Repeater(functools.partial(Handlers().method), 0.1)


def test_callable_object_is_rejected():
    with pytest.raises(TypeError, match='unsupported handler'):
        Repeater(CallableHandler(), 0.1)


def test_non_weakreferenceable_object_is_rejected():
    with pytest.raises(TypeError, match='does not support weak references'):
        Repeater(SlotsValue().method, 0.1)  # no __weakref__, so WeakMethod would raise TypeError


def test_handler_name_falls_back_when_qualname_is_missing():
    assert _handler_name(plain_function) == 'plain_function'  # functions/methods keep their qualname
    partial = functools.partial(Handlers().method)
    assert _handler_name(partial) == repr(partial)  # partials and callables have no __qualname__


def test_weak_handler_does_not_keep_object_alive():
    handlers = Handlers()
    reference = weakref.ref(handlers)
    handler = _prepare_handler(handlers.method)
    assert isinstance(handler, _WeakHandler)

    del handlers
    gc.collect()

    assert reference() is None  # object collected despite the live handler
    assert not handler.alive  # handler reports its object is gone
    assert handler() is None  # calling it is a safe no-op


async def test_collected_object_does_not_start_orphan_task_after_startup():
    # NOTE: a repeater created pre-startup defers its start; if the object dies first,
    # the replayed start (during startup) must see the dead handler and not launch an orphan task.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    assert not _state.startup_finished

    handlers = Handlers()
    repeater = rosys.on_repeat(handlers.method, 0.01)
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


async def test_never_stored_object_logs_warning(rosys_log: pytest.LogCaptureFixture):
    calls: list[float] = []
    repeater = rosys.on_repeat(Ticker(calls).step, 0.1)  # NOTE: the Ticker is not stored anywhere

    await forward(0.5)

    assert not repeater.running
    assert not calls  # never ticked
    assert any('will never be called' in record.message for record in rosys_log.records)


async def test_object_collected_later_logs_debug_instead_of_warning(rosys_log: pytest.LogCaptureFixture):
    rosys_log.set_level(logging.DEBUG, logger='rosys.core')
    calls: list[float] = []
    ticker = Ticker(calls)
    repeater = rosys.on_repeat(ticker.step, 0.1)

    await forward(0.35)
    del ticker
    gc.collect()
    await forward(0.5)

    assert not repeater.running
    assert not any('will never be called' in record.message for record in rosys_log.records)
    assert any('garbage-collected' in record.message and record.levelno == logging.DEBUG
               for record in rosys_log.records)


@pytest.mark.usefixtures('rosys_integration')
async def test_repeater_stops_when_object_is_collected():
    calls: list[float] = []
    ticker = Ticker(calls)
    repeater = rosys.on_repeat(ticker.step, 0.1)
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
    assert repeater._task is None  # pylint: disable=protected-access  # ...and cleared the reference, like stop()
    assert len(calls) == calls_while_alive  # and stopped ticking after collection
