import pytest


@pytest.mark.asyncio
async def test_clock(world):

    await world.simulate(seconds=1.0)
    assert world.clock.time > 1.0

    await world.simulate(seconds=1.0)
    assert world.clock.time > 2.0
