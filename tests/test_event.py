import pytest
from rosys import event


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

    class Actor():
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
