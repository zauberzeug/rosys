import pytest
from rosys.runtime import Runtime


@pytest.mark.asyncio
async def test_time(runtime: Runtime):
    start = runtime.world.time

    await runtime.forward(seconds=1.0)
    assert runtime.world.time > start + 1.0

    await runtime.forward(seconds=1.0)
    assert runtime.world.time > start + 2.0
