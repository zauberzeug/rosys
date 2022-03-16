import pytest
import numpy as np
from rosys.automations import drive_path
from rosys.world import Obstacle, Point, Pose
from rosys.test import TestRuntime, assert_point


@pytest.mark.asyncio
async def test_basic_path_planning(runtime: TestRuntime):
    await runtime.forward(1.0)

    goal = Pose(x=1.0, y=1.0)
    path = await runtime.path_planner.search_async(goal=goal, timeout=3.0)
    assert_point(path[-1].spline.end, goal.point)

    goal = Pose(x=3.0, y=2.0)
    path = await runtime.path_planner.search_async(goal=goal, timeout=1.0)
    assert_point(path[-1].spline.end, goal.point)


@pytest.mark.asyncio
async def test_driving_to_planned_point(runtime: TestRuntime):
    await runtime.forward(1.0)
    path = await runtime.path_planner.search_async(goal=Pose(x=5, y=2), timeout=3.0)
    runtime.automator.start(drive_path(runtime.world, runtime.hardware, path))
    await runtime.forward(x=5, y=2, tolerance=0.15)


@pytest.mark.asyncio
async def test_planning_to_problematic_location(runtime: TestRuntime):
    await runtime.forward(1.0)
    await runtime.path_planner.search_async(goal=Pose(x=2.250, y=1.299, yaw=np.deg2rad(-60.0)), timeout=3.0)


@pytest.mark.asyncio
async def test_not_finding_a_path(runtime: TestRuntime):
    await runtime.forward(1.0)
    id = 'o1'
    p = Point(x=3, y=0)
    runtime.world.obstacles[id] = Obstacle(id=id, outline=[
        Point(x=p.x-0.5, y=p.y-0.5),
        Point(x=p.x+0.5, y=p.y-0.5),
        Point(x=p.x+0.5, y=p.y+0.5),
        Point(x=p.x-0.5, y=p.y+0.5),
    ])
    with pytest.raises(TimeoutError):
        await runtime.path_planner.search_async(goal=Pose(x=3, y=0), timeout=1.0)
