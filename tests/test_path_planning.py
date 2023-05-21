import asyncio
import time
import uuid

import numpy as np
import pytest

from rosys.automation import Automator
from rosys.driving import Driver
from rosys.geometry import Point, Pose, Prism, Spline
from rosys.hardware import Robot
from rosys.pathplanning import Obstacle, PathPlanner
from rosys.pathplanning.delaunay_planner import DelaunayPlanner
from rosys.test import assert_point, forward


def create_obstacle(*, x: float, y: float, radius: float = 0.5) -> Obstacle:
    return Obstacle(id=str(uuid.uuid4()), outline=[
        Point(x=x-radius, y=y-radius),
        Point(x=x+radius, y=y-radius),
        Point(x=x+radius, y=y+radius),
        Point(x=x-radius, y=y+radius),
    ])


async def test_basic_path_planning(path_planner: PathPlanner) -> None:
    await forward(1.0)

    goal = Pose(x=1.0, y=1.0)
    path = await path_planner.search(start=Pose(), goal=goal, timeout=3.0)
    assert_point(path[-1].spline.end, goal.point)

    goal = Pose(x=3.0, y=2.0)
    path = await path_planner.search(start=Pose(), goal=goal, timeout=1.0)
    assert_point(path[-1].spline.end, goal.point)


async def test_driving_to_planned_point(path_planner: PathPlanner, driver: Driver, automator: Automator, robot: Robot) -> None:
    await forward(1.0)
    path = await path_planner.search(start=Pose(), goal=Pose(x=5, y=2), timeout=3.0)
    automator.start(driver.drive_path(path))
    await forward(x=5, y=2, tolerance=0.15)


async def test_planning_to_problematic_location(path_planner: PathPlanner) -> None:
    await forward(1.0)
    await path_planner.search(start=Pose(), goal=Pose(x=2.250, y=1.299, yaw=np.deg2rad(-60.0)), timeout=3.0)


async def test_not_finding_a_path(path_planner: PathPlanner) -> None:
    await forward(1.0)
    obstacle = create_obstacle(x=2, y=0)
    path_planner.obstacles[obstacle.id] = obstacle
    with pytest.raises(TimeoutError):
        await path_planner.search(start=Pose(), goal=Pose(x=1, y=0), timeout=0.1)
    with pytest.raises(RuntimeError):
        await path_planner.search(start=Pose(), goal=Pose(x=2, y=0))


async def test_test_spline(path_planner: PathPlanner) -> None:
    await forward(1.0)

    spline = Spline.from_poses(Pose(x=0, y=0), Pose(x=2, y=1))
    assert await path_planner.test_spline(spline) == False

    obstacle = create_obstacle(x=2, y=1)
    path_planner.obstacles[obstacle.id] = obstacle
    assert await path_planner.test_spline(spline) == True


def test_grow_map(shape: Prism) -> None:
    planner = DelaunayPlanner(shape.outline)
    assert planner.obstacle_map is None

    start = Pose(x=0, y=0)
    goal = Pose(x=2, y=1)
    planner.update_map([], [], [start, goal], time.time() + 3.0)
    path = planner.search(start, goal)
    assert path is not None
    assert planner.obstacle_map.grid.bbox == pytest.approx((-1.2, -1.2, 4.4, 3.4))

    planner.grow_map([Point(x=5, y=0)], time.time() + 3.0)
    assert planner.obstacle_map.grid.bbox == pytest.approx((-2.4, -2.4, 8.6, 5.8))


async def test_overlapping_commands(path_planner: PathPlanner) -> None:
    await forward(1.0)

    start = Pose()
    end = Pose(x=10.0, y=1.0)
    spline = Spline.from_poses(start, end)

    task1 = asyncio.create_task(path_planner.search(start=Pose(), goal=end), name='search')
    await asyncio.sleep(0.2)
    task2 = asyncio.create_task(path_planner.test_spline(spline), name='test')
    path, test = await asyncio.gather(task1, task2)
    assert isinstance(path, list)
    assert isinstance(test, bool)
