import pytest
from rosys.test import TestRuntime


@pytest.mark.asyncio
async def test_time(runtime: TestRuntime):
    await runtime.forward(until=1.0)
    assert runtime.world.time > 1.0

    await runtime.forward(seconds=1.0)
    assert runtime.world.time > 2.0

    await runtime.forward(3.0)
    assert runtime.world.time > 3.0
