import pytest
from rosys.test import TestRuntime, assert_pose
from rosys.world import Pose


@pytest.mark.asyncio
async def test_not_finding_a_path(runtime: TestRuntime):
    await runtime.forward(1.0)
    with pytest.raises(TimeoutError):
        await runtime.path_planner.search_async(goal=Pose(x=1.0), timeout=0.1)


@pytest.mark.asyncio
async def test_drive(runtime: TestRuntime):
    await runtime.hardware.drive(1.0, 0.0)
    await runtime.forward(seconds=1.0)
    assert_pose(1.0, 0, deg=0)
