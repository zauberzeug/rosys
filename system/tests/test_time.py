import pytest
from runtime import Runtime


@pytest.mark.asyncio
async def test_time(runtime: Runtime):

    await runtime.run(seconds=1.0)
    assert runtime.world.time > 1.0

    await runtime.run(seconds=1.0)
    assert runtime.world.time > 2.0
