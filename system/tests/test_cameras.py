import pytest
from runtime import Runtime


@pytest.mark.asyncio
async def test_drive(runtime: Runtime):
    await runtime.run(1)
    assert len(runtime.world.cameras) == 1
