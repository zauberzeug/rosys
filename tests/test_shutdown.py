import asyncio
import gc
import weakref

import pytest
from nicegui import core

import rosys
from rosys.automation import Automator
from rosys.hardware import WheelsSimulation
from rosys.rosys import shutdown_handlers


@pytest.mark.usefixtures('rosys_integration')
async def test_on_shutdown_does_not_keep_a_discarded_module_alive():
    wheels = WheelsSimulation()  # registers rosys.on_shutdown(self.stop) in __init__
    reference = weakref.ref(wheels)

    del wheels
    gc.collect()

    assert reference() is None  # the shutdown registry does not pin the module


@pytest.mark.usefixtures('rosys_integration')
async def test_on_shutdown_does_not_keep_a_discarded_automator_alive():
    automator = Automator(None)  # registers a shutdown handler for its own teardown
    reference = weakref.ref(automator)

    del automator
    gc.collect()

    assert reference() is None


async def test_weak_shutdown_handler_is_still_awaited_while_its_object_lives():
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    calls: list[str] = []

    class Stoppable:
        async def stop(self) -> None:  # async, so this also covers awaiting through the weak wrapper
            calls.append('stopped')

    stoppable = Stoppable()  # kept alive, so its handler must still run
    try:
        rosys.on_shutdown(stoppable.stop)
        await rosys.startup()
        await rosys.shutdown()
        assert calls == ['stopped']
    finally:
        rosys.reset_after_test()


@pytest.mark.usefixtures('rosys_integration')
async def test_shutdown_handlers_do_not_accumulate_for_discarded_modules():
    count = len(shutdown_handlers)

    for _ in range(5):
        WheelsSimulation()  # each registers a shutdown handler and is discarded right away
    gc.collect()

    assert len(shutdown_handlers) == count  # the dead entries removed themselves


async def test_a_module_dying_during_shutdown_does_not_skip_another_handler():
    # NOTE: a dying module drops its own registry entry, which must not shift the handlers still to be invoked.
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    calls: list[str] = []

    class Victim:
        async def stop(self) -> None:
            calls.append('victim')

    class Dropper:
        def __init__(self, box: list) -> None:
            self.box = box

        async def stop(self) -> None:
            calls.append('dropper')
            self.box.clear()  # the victim dies while the registry is being iterated
            gc.collect()

    class Last:
        async def stop(self) -> None:
            calls.append('last')

    box = [Victim()]
    dropper, last = Dropper(box), Last()
    try:
        rosys.on_shutdown(box[0].stop)
        rosys.on_shutdown(dropper.stop)
        rosys.on_shutdown(last.stop)
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
