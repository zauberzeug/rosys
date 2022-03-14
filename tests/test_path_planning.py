import pytest
import numpy as np
import rosys
from rosys.automations import drive_path
from rosys.world import Obstacle, Point, Pose
from rosys.test import TestRuntime


@pytest.mark.asyncio
async def test_driving_to_planned_point(runtime: TestRuntime):
    planner = rosys.pathplanning.Planner(runtime.world)
    path = await planner.search_async(goal=Pose(x=5, y=2), timeout=3.0)
    runtime.automator.start(drive_path(runtime.world, runtime.hardware, path))
    await runtime.forward(x=5, y=2, tolerance=0.15)


@pytest.mark.asyncio
async def test_planning_to_problematic_location(runtime: TestRuntime):
    planner = rosys.pathplanning.Planner(runtime.world)
    planner.search(goal=Pose(x=2.250, y=1.299, yaw=np.deg2rad(-60.0)), timeout=3.0)


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
