import pytest


@pytest.mark.asyncio
async def test_time(world):

    await world.run(seconds=1.0)
    assert world.robot.machine.time > 1.0

    await world.run(seconds=1.0)
    assert world.robot.machine.time > 2.0
