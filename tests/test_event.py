import pytest
from nicegui import Event

import rosys
from rosys.testing import forward

TEST_EVENT = Event[int]()


async def test_register_and_unregister():
    def handler(number: int) -> None: numbers.append(number)
    numbers = []
    await TEST_EVENT.call(1)
    assert numbers == []

    TEST_EVENT.subscribe(handler)
    await TEST_EVENT.call(2)
    assert numbers == [2]

    TEST_EVENT.unsubscribe(handler)
    await TEST_EVENT.call(3)
    assert numbers == [2]


async def test_registering_multiple_handlers():
    def handler1(number: int) -> None: numbers.append(number)
    def handler2(number: int) -> None: numbers.append(number)
    numbers = []
    TEST_EVENT.subscribe(handler1)
    TEST_EVENT.subscribe(handler2)
    await TEST_EVENT.call(42)
    assert numbers == [42, 42], 'all registered handlers have been called'


async def test_registering_same_handler_multiple_times():
    def handler(number: int) -> None: numbers.append(number)
    numbers = []
    TEST_EVENT.subscribe(handler)
    TEST_EVENT.subscribe(handler)
    await TEST_EVENT.call(42)
    assert numbers == [42], 'the same handler should only be registered once'


async def test_registering_lambdas():
    numbers = []
    TEST_EVENT.subscribe(lambda number: numbers.append(number))
    await TEST_EVENT.call(42)
    assert numbers == [42]


@pytest.mark.usefixtures('rosys_integration')
async def test_fire_and_forget_with_emit():

    async def handle_event(number: int) -> None:
        await rosys.sleep(1)
        numbers.append(number)
        await rosys.sleep(1)
        raise Exception('some failure which should be detected even when using "fire and forget"')

    numbers = []
    TEST_EVENT.subscribe(handle_event)
    TEST_EVENT.emit(42)
    await forward(0.5)
    assert numbers == []
    await forward(1.0)
    assert numbers == [42]
    with pytest.raises(RuntimeError) as ex_info:
        await forward(1.0)
    assert ex_info.value.__cause__ is not None
    assert 'some failure' in str(ex_info.value.__cause__)
