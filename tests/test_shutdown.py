import asyncio
import gc
import weakref
from collections.abc import Callable

import pytest
from nicegui import core

import rosys
from rosys.automation import Automator
from rosys.hardware import WheelsSimulation
from rosys.rosys import shutdown_handlers


@pytest.mark.usefixtures('rosys_integration')
@pytest.mark.parametrize('create_module', [WheelsSimulation, lambda: Automator(None)], ids=['wheels', 'automator'])
async def test_on_shutdown_does_not_keep_a_discarded_module_alive(create_module: Callable):
    count = len(shutdown_handlers)
    module = create_module()  # registers a shutdown handler for its own teardown in __init__
    reference = weakref.ref(module)

    del module
    gc.collect()

    assert reference() is None  # the shutdown registry does not pin the module
    assert len(shutdown_handlers) == count  # ...and the dead entry removed itself


async def test_weak_shutdown_handler_is_still_awaited_while_its_object_lives():
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    wheels = WheelsSimulation()  # kept alive, so its async stop handler must still run at shutdown
    try:
        await rosys.startup()
        await wheels.drive(1.0, 0.0)
        assert wheels.linear_target_speed == 1.0
        await rosys.shutdown()
        assert wheels.linear_target_speed == 0.0  # stop() was awaited through the weak wrapper
    finally:
        rosys.reset_after_test()


async def test_a_handler_removed_during_shutdown_does_not_skip_another_handler():
    # NOTE: a module dying mid-shutdown removes its own entry (see _forget_shutdown_handler),
    # which must not shift the handlers still to be invoked.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    calls: list[str] = []

    def victim() -> None:
        calls.append('victim')

    def dropper() -> None:
        calls.append('dropper')
        shutdown_handlers.remove(victim)  # what the finalizer of a dying module does to its own entry

    def last() -> None:
        calls.append('last')

    try:
        rosys.on_shutdown(victim)
        rosys.on_shutdown(dropper)
        rosys.on_shutdown(last)
        await rosys.startup()
        await rosys.shutdown()
        assert calls == ['victim', 'dropper', 'last']
    finally:
        rosys.reset_after_test()


@pytest.mark.usefixtures('rosys_integration')
async def test_a_handler_registered_from_a_temporary_leaves_no_dead_entry():
    count = len(shutdown_handlers)

    rosys.on_shutdown(WheelsSimulation().stop)  # NOTE: the object is gone as soon as the call returns
    gc.collect()

    assert len(shutdown_handlers) == count
