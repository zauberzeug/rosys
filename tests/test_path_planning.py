import pytest
from rosys.test import assert_pose, TestRuntime
from rosys.automations import drive_path
from rosys.world import PathSegment, Pose, Point3d, Spline
import rosys


@pytest.mark.asyncio
async def test_planning_to_point(runtime: TestRuntime):
    planner = rosys.pathplanning.Planner(runtime.world)
    planner.search(goal=Pose(x=5, y=2), timeout=3.0)
    path = [PathSegment(spline=step.spline, backward=step.backward) for step in planner.path]
    runtime.automator.replace(drive_path(runtime.world, runtime.hardware, path))
    await runtime.resume()
    await runtime.forward(12)
    assert_pose(5, 2, deg=0, linear_tolerance=0.15, deg_tolerance=3)
