import pytest
from rosys.test import assert_pose, TestRuntime
from rosys.automations import drive_path
from rosys.world import PathSegment, Pose
import rosys
from rosys.world.obstacle import Obstacle
from rosys.world.point import Point


@pytest.mark.asyncio
async def test_driving_to_planned_point(runtime: TestRuntime):
    planner = rosys.pathplanning.Planner(runtime.world)
    planner.search(goal=Pose(x=5, y=2), timeout=3.0)
    path = [PathSegment(spline=step.spline, backward=step.backward) for step in planner.path]
    runtime.automator.replace(drive_path(runtime.world, runtime.hardware, path))
    await runtime.resume()
    await runtime.forward(12)
    assert_pose(5, 2, deg=0, linear_tolerance=0.15, deg_tolerance=3)


@pytest.mark.asyncio
async def test_planning_to_problematic_location(runtime: TestRuntime):
    planner = rosys.pathplanning.Planner(runtime.world)
    planner.search(goal=Pose(x=2, y=2), timeout=3.0)


@pytest.mark.asyncio
async def test_not_finding_a_path(runtime: TestRuntime):
    id = 'o1'
    p = Point(x=3, y=0)
    runtime.world.obstacles[id] = Obstacle(id=id, outline=[
        Point(x=p.x-0.5, y=p.y-0.5),
        Point(x=p.x+0.5, y=p.y-0.5),
        Point(x=p.x+0.5, y=p.y+0.5),
        Point(x=p.x-0.5, y=p.y+0.5),
    ])
    planner = rosys.pathplanning.Planner(runtime.world)
    with pytest.raises(TimeoutError):
        planner.search(goal=Pose(x=3, y=0), timeout=1.0)
