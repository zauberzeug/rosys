import pytest
from rosys import event, runtime
from rosys.test import forward


@pytest.mark.asyncio
async def test_registering_multiple_handlers():
    def handler1(number: int) -> None: numbers.append(number)
    def handler2(number: int) -> None: numbers.append(number)
    numbers = []
    runtime.NEW_NOTIFICATION.register(handler1)
    runtime.NEW_NOTIFICATION.register(handler2)
    await runtime.NEW_NOTIFICATION.call(42)
    assert numbers == [42, 42], 'all registered handlers have been called'


@pytest.mark.asyncio
async def test_registering_same_handler_multiple_times():
    def handler(number: int) -> None: numbers.append(number)
    numbers = []
    runtime.NEW_NOTIFICATION.register(handler)
    runtime.NEW_NOTIFICATION.register(handler)
    await runtime.NEW_NOTIFICATION.call(42)
    assert numbers == [42], 'the same handler should only be registered once'


@pytest.mark.asyncio
async def test_automatic_deregistering_if_reference_is_unused():

    class Actor:
        def __init__(self) -> None:
            runtime.NEW_NOTIFICATION.register(self.handler)

        def handler(self, number: int) -> None:
            numbers.append(number)

    numbers = []
    actor = Actor()
    await runtime.NEW_NOTIFICATION.call(42)
    assert numbers == [42]
    actor = Actor()
    await runtime.NEW_NOTIFICATION.call(42)
    assert numbers == [42, 42]


@pytest.mark.asyncio
async def test_registering_lambdas():
    numbers = []
    runtime.NEW_NOTIFICATION.register(lambda number: numbers.append(number))
    await runtime.NEW_NOTIFICATION.call(42)
    assert numbers == [42]


@pytest.mark.asyncio
async def test_fire_and_forget_with_emit():

    class TestActor:

        async def handle_event(self, number: int) -> None:
            await runtime.sleep(1)
            numbers.append(number)
            await runtime.sleep(1)
            raise Exception('some failure which should be detected even when using "fire and forget"')

    numbers = []
    actor = TestActor()
    runtime.NEW_NOTIFICATION.register(actor.handle_event)
    runtime.NEW_NOTIFICATION.emit(42)
    await forward(0.5)
    assert numbers == []
    await forward(1.0)
    assert numbers == [42]
    with pytest.raises(RuntimeError) as ex_info:
        await forward(1.0)
    assert ex_info.value.__cause__ is not None
    assert 'some failure' in str(ex_info.value.__cause__)
