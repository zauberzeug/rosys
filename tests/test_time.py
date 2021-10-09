import pytest
from rosys.runtime import Runtime


@pytest.mark.asyncio
async def test_time(runtime: Runtime):
    start = runtime.world.time

    await runtime.advance_time(seconds=1.0)
    assert runtime.world.time > start + 1.0

    await runtime.advance_time(seconds=1.0)
    assert runtime.world.time > start + 2.0
