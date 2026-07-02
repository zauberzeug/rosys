import asyncio

import pytest
from nicegui import Client, core
from nicegui.page import page

import rosys
from rosys.rosys import Repeater, _state, startup_handlers
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

    client.delete()
    await forward(0.35)
    assert repeater.running  # the default scope is not affected by client deletion


@pytest.mark.usefixtures('rosys_integration')
async def test_client_scoped_repeater_stops_when_client_is_deleted():
    ticker = Ticker()
    client = Client(page('/client-scope-test'), request=None)
    with client:
        repeater = rosys.on_repeat(ticker.step, 0.1, scope='client')

    await forward(0.35)
    assert repeater.running
    calls_while_alive = len(ticker.calls)
    assert calls_while_alive >= 1  # the repeater was actually ticking

    client.delete()
    assert not repeater.running  # the client teardown stopped the repeater
    assert repeater.handler is None  # the handler is released so the deleted client does not pin its owner
    repeater.start()
    assert not repeater.running  # a client-scoped repeater cannot be revived after its client is deleted
    await forward(0.5)
    assert len(ticker.calls) == calls_while_alive  # no more ticks after deletion


@pytest.mark.usefixtures('rosys_integration')
async def test_client_scope_outside_ui_context_raises():
    ticker = Ticker()
    with pytest.raises(RuntimeError):
        rosys.on_repeat(ticker.step, 0.1, scope='client')


@pytest.mark.usefixtures('rosys_integration')
async def test_client_scope_binds_to_script_mode_client():
    # NOTE: NiceGUI deletes the script-mode client before startup;
    # a client-scoped repeater binds to it and dies with it, like with any other client.
    ticker = Ticker()
    client = Client(page('/script-mode-test'), request=None)
    core.script_client = client
    try:
        with client:
            repeater = rosys.on_repeat(ticker.step, 0.1, scope='client')
        client.delete()
    finally:
        core.script_client = None

    assert not repeater.running  # died with the script-mode client


async def test_stop_before_startup_prevents_deferred_start():
    # NOTE: a repeater created pre-startup defers its start; stopping it before startup
    # must remove the deferred start so the repeater is not revived when startup replays the handlers.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    try:
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
    finally:
        rosys.reset_after_test()


async def test_stopping_repeater_during_startup_does_not_skip_other_handlers():
    # NOTE: Repeater.stop() removes a deferred start from startup_handlers;
    # startup() must iterate a snapshot so this mutation does not skip the following handler.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    try:
        ticker = Ticker()
        repeater = rosys.on_repeat(ticker.step, 0.01)
        events: list[str] = []

        def stopper() -> None:
            repeater.stop()
            events.append('stopper')

        def victim() -> None:
            events.append('victim')

        rosys.on_startup(stopper)
        rosys.on_startup(victim)

        await rosys.startup()

        assert events == ['stopper', 'victim']  # the mutation must not skip the next startup handler
        assert not repeater.running

        await rosys.shutdown()
    finally:
        rosys.reset_after_test()


async def test_stopping_repeater_during_startup_does_not_revive_it():
    # NOTE: flipped ordering of the test above: when the stopping handler precedes the deferred start,
    # startup() must not invoke the stale snapshot entry and revive the stopped repeater.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    try:
        ticker = Ticker()
        repeaters: list[Repeater] = []

        def stopper() -> None:
            repeaters[0].stop()

        rosys.on_startup(stopper)  # registered before the repeater, so its deferred start comes later
        repeaters.append(rosys.on_repeat(ticker.step, 0.01))
        assert startup_handlers.index(stopper) < startup_handlers.index(repeaters[0].start)

        await rosys.startup()

        assert not repeaters[0].running  # the stopped repeater was not revived by the snapshot

        await rosys.shutdown()
    finally:
        rosys.reset_after_test()
