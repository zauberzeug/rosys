import asyncio
import time
import uuid

import numpy as np
import pytest
from rosys.actors.pathplanning.delaunay_planner import DelaunayPlanner
from rosys.test import assert_point
from rosys.world import Obstacle, Point, Pose, Spline

from conftest import TestRuntime


def create_obstacle(*, x: float, y: float, radius: float = 0.5) -> Obstacle:
    return Obstacle(id=str(uuid.uuid4()), outline=[
        Point(x=x-radius, y=y-radius),
        Point(x=x+radius, y=y-radius),
        Point(x=x+radius, y=y+radius),
        Point(x=x-radius, y=y+radius),
    ])


@pytest.mark.asyncio
async def test_basic_path_planning(runtime: TestRuntime):
    await runtime.forward(1.0)

    goal = Pose(x=1.0, y=1.0)
    path = await runtime.path_planner.search(goal=goal, timeout=3.0)
    assert_point(path[-1].spline.end, goal.point)

    goal = Pose(x=3.0, y=2.0)
    path = await runtime.path_planner.search(goal=goal, timeout=1.0)
    assert_point(path[-1].spline.end, goal.point)


@pytest.mark.asyncio
async def test_driving_to_planned_point(runtime: TestRuntime):
    await runtime.forward(1.0)
    path = await runtime.path_planner.search(goal=Pose(x=5, y=2), timeout=3.0)
    runtime.automator.start(drive_path(runtime.world, runtime.hardware, path))
    await runtime.forward(x=5, y=2, tolerance=0.15)


@pytest.mark.asyncio
async def test_planning_to_problematic_location(runtime: TestRuntime):
    await runtime.forward(1.0)
    await runtime.path_planner.search(goal=Pose(x=2.250, y=1.299, yaw=np.deg2rad(-60.0)), timeout=3.0)


@pytest.mark.asyncio
async def test_not_finding_a_path(runtime: TestRuntime):
    await runtime.forward(1.0)
    obstacle = create_obstacle(x=2, y=0)
    runtime.world.obstacles[obstacle.id] = obstacle
    with pytest.raises(TimeoutError):
        await runtime.path_planner.search(goal=Pose(x=1, y=0), timeout=0.1)
    with pytest.raises(RuntimeError):
        await runtime.path_planner.search(goal=Pose(x=2, y=0))


@pytest.mark.asyncio
async def test_test_spline(runtime: TestRuntime):
    await runtime.forward(1.0)

    spline = Spline.from_poses(Pose(x=0, y=0), Pose(x=2, y=1))
    assert await runtime.path_planner.test_spline(spline) == False

    obstacle = create_obstacle(x=2, y=1)
    runtime.world.obstacles[obstacle.id] = obstacle
    assert await runtime.path_planner.test_spline(spline) == True


def test_grow_map(runtime: TestRuntime):
    planner = DelaunayPlanner(runtime.world.robot.shape.outline)
    assert planner.obstacle_map is None

    start = Pose(x=0, y=0)
    goal = Pose(x=2, y=1)
    planner.update_map(runtime.world.areas.values(), runtime.world.obstacles.values(), [start, goal], time.time() + 3.0)
    path = planner.search(start, goal)
    assert path is not None
    assert planner.obstacle_map.grid.bbox == pytest.approx((-1.2, -1.2, 4.4, 3.4))

    planner.grow_map([Point(x=5, y=0)], time.time() + 3.0)
    assert planner.obstacle_map.grid.bbox == pytest.approx((-2.4, -2.4, 8.6, 5.8))


@pytest.mark.asyncio
async def test_overlapping_commands(runtime: TestRuntime):
    await runtime.forward(1.0)

    start = Pose()
    end = Pose(x=10.0, y=1.0)
    spline = Spline.from_poses(start, end)

    task1 = asyncio.create_task(runtime.path_planner.search(goal=end), name='search')
    await asyncio.sleep(0.2)
    task2 = asyncio.create_task(runtime.path_planner.test_spline(spline), name='test')
    path, test = await asyncio.gather(task1, task2)
    assert isinstance(path, list)
    assert isinstance(test, bool)
