import asyncio

import pytest
from nicegui import Client, core
from nicegui.page import page

import rosys
from rosys.rosys import _state, startup_handlers
from rosys.testing import forward


class Ticker:

    def __init__(self) -> None:
        self.calls: list[float] = []

    def step(self) -> None:
        self.calls.append(rosys.time())


@pytest.mark.usefixtures('rosys_integration')
async def test_repeater_can_restart_after_stop():
    ticker = Ticker()
    repeater = rosys.on_repeat(ticker.step, 0.1)

    await forward(0.35)
    assert repeater.running
    assert len(ticker.calls) >= 1

    repeater.stop()
    assert not repeater.running
    calls_after_stop = len(ticker.calls)
    await forward(0.5)
    assert len(ticker.calls) == calls_after_stop  # stopped: no more ticks

    repeater.start()
    assert repeater.running  # a plain stop() must not permanently disable restart
    await forward(0.35)
    assert len(ticker.calls) > calls_after_stop  # restarted: ticking again
    assert repeater.running  # the old task's done-callback must not clear the new task


@pytest.mark.usefixtures('rosys_integration')
async def test_app_scoped_repeater_ignores_ui_context():
    ticker = Ticker()
    client = Client(page('/app-scope-test'), request=None)
    with client:
        repeater = rosys.on_repeat(ticker.step, 0.1)
    assert repeater._client is None  # pylint: disable=protected-access

    client.delete()
    await forward(0.35)
    assert repeater.running  # the default scope is not affected by client deletion


@pytest.mark.usefixtures('rosys_integration')
async def test_client_scoped_repeater_stops_when_client_is_deleted():
    ticker = Ticker()
    client = Client(page('/client-scope-test'), request=None)
    with client:
        repeater = rosys.on_repeat(ticker.step, 0.1, scope='auto')
    assert repeater._client is client  # pylint: disable=protected-access

    await forward(0.35)
    assert repeater.running
    calls_while_alive = len(ticker.calls)
    assert calls_while_alive >= 1  # the repeater was actually ticking

    client.delete()
    assert not repeater.running  # the client teardown stopped the repeater
    await forward(0.5)
    assert len(ticker.calls) == calls_while_alive  # no more ticks after deletion


@pytest.mark.usefixtures('rosys_integration')
async def test_auto_scope_outside_ui_context_falls_back_to_app_scope():
    ticker = Ticker()
    repeater = rosys.on_repeat(ticker.step, 0.1, scope='auto')
    assert repeater._client is None  # pylint: disable=protected-access
    await forward(0.25)
    assert repeater.running


@pytest.mark.usefixtures('rosys_integration')
async def test_client_scope_outside_ui_context_raises():
    ticker = Ticker()
    with pytest.raises(RuntimeError):
        rosys.on_repeat(ticker.step, 0.1, scope='client')


async def test_stop_before_startup_prevents_deferred_start():
    # NOTE: a repeater created pre-startup defers its start; stopping it before startup
    # must remove the deferred start so the repeater is not revived when startup replays the handlers.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    assert not _state.startup_finished

    ticker = Ticker()
    repeater = rosys.on_repeat(ticker.step, 0.01)
    assert not repeater.running  # start was deferred, not launched
    assert repeater.start in startup_handlers

    repeater.stop()
    assert repeater.start not in startup_handlers

    await rosys.startup()  # replays the deferred start handlers

    assert not repeater.running  # the stopped repeater was not revived

    await rosys.shutdown()
    rosys.reset_after_test()
