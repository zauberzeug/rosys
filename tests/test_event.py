import pytest
import rosys
from rosys.event import Event
from rosys.test import forward

TEST_EVENT = Event()


@pytest.mark.asyncio
async def test_registering_multiple_handlers():
    def handler1(number: int) -> None: numbers.append(number)
    def handler2(number: int) -> None: numbers.append(number)
    numbers = []
    TEST_EVENT.register(handler1)
    TEST_EVENT.register(handler2)
    await TEST_EVENT.call(42)
    assert numbers == [42, 42], 'all registered handlers have been called'


@pytest.mark.asyncio
async def test_registering_same_handler_multiple_times():
    def handler(number: int) -> None: numbers.append(number)
    numbers = []
    TEST_EVENT.register(handler)
    TEST_EVENT.register(handler)
    await TEST_EVENT.call(42)
    assert numbers == [42], 'the same handler should only be registered once'


@pytest.mark.asyncio
async def test_registering_lambdas():
    numbers = []
    TEST_EVENT.register(lambda number: numbers.append(number))
    await TEST_EVENT.call(42)
    assert numbers == [42]


@pytest.mark.asyncio
async def test_fire_and_forget_with_emit():

    class TestActor:

        async def handle_event(self, number: int) -> None:
            await rosys.sleep(1)
            numbers.append(number)
            await rosys.sleep(1)
            raise Exception('some failure which should be detected even when using "fire and forget"')

    numbers = []
    actor = TestActor()
    TEST_EVENT.register(actor.handle_event)
    TEST_EVENT.emit(42)
    await forward(0.5)
    assert numbers == []
    await forward(1.0)
    assert numbers == [42]
    with pytest.raises(RuntimeError) as ex_info:
        await forward(1.0)
    assert ex_info.value.__cause__ is not None
    assert 'some failure' in str(ex_info.value.__cause__)
