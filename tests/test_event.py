import pytest
from rosys import event, runtime
from rosys.test import forward


@pytest.mark.asyncio
async def test_registering_multiple_handlers():

    def handler1(param):
        calls.append(param)

    def handler2(param):
        calls.append(param)

    calls = []
    event.register('test', handler1)
    event.register('test', handler2)
    await event.call('test', 1)
    assert len(calls) == 2, 'all registered handlers are called'
    assert calls[0] == 1
    assert calls[1] == 1


@pytest.mark.asyncio
async def test_registering_same_handler_multiple_times():

    def handler(param):
        calls.append(param)

    calls = []
    event.register('test', handler)
    event.register('test', handler)
    await event.call('test', 1)
    assert len(calls) == 1, 'the same handler should only be registered once'
    assert calls[0] == 1


@pytest.mark.asyncio
async def test_automatic_deregistering_if_reference_is_unused():

    class Actor:
        def __init__(self) -> None:
            event.register('test', self.handler)

        def handler(self, param):
            calls.append(param)

    calls = []
    actor = Actor()
    await event.call('test', 1)
    assert len(calls) == 1
    assert calls[0] == 1
    actor = Actor()
    await event.call('test', 2)
    assert len(calls) == 2
    assert calls[1] == 2


@pytest.mark.asyncio
async def test_registering_lambdas():
    calls = []
    event.register('test', lambda x: calls.append(x))
    await event.call('test', 1)
    assert len(calls) == 1


def test_id_enum_definition():
    assert event.Id.NEW_MACHINE_DATA.value == 'NEW_MACHINE_DATA', 'value should be an readable string'
    assert 'machine data' in event.Id.NEW_MACHINE_DATA.__doc__, '__doc__ should have a description'


@pytest.mark.asyncio
async def test_fire_and_forget_with_emit():

    class TestActor:

        async def handle_event(self, param):
            await runtime.sleep(1)
            calls.append(param)
            await runtime.sleep(1)
            raise Exception('some failure which should be detected even when using "fire and forget"')

    calls = []

    actor = TestActor()
    event.register(event.Id.NEW_NOTIFICATION, actor.handle_event)
    event.emit(event.Id.NEW_NOTIFICATION, 42)
    await forward(0.5)
    assert len(calls) == 0
    await forward(1.0)
    assert calls[0] == 42
    with pytest.raises(RuntimeError) as ex_info:
        await forward(1.0)
    assert ex_info.value.__cause__ is not None
    assert 'some failure' in str(ex_info.value.__cause__)
