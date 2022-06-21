import abc
import logging
import uuid
from dataclasses import dataclass, field
from multiprocessing import Process
from multiprocessing.connection import Connection
from typing import Any

import icecream

from ...world import Area, Point, Pose, Spline
from .delaunay_planner import DelaunayPlanner
from .obstacle_map import Obstacle


@dataclass
class PlannerCommand(abc.ABC):
    id: str = field(init=False, repr=False)
    deadline: float

    def __post_init__(self):
        self.id = str(uuid.uuid4())


@dataclass
class PlannerSearchCommand(PlannerCommand):
    areas: list[Area]
    obstacles: list[Obstacle]
    start: Pose
    goal: Pose


@dataclass
class PlannerGrowMapCommand(PlannerCommand):
    points: list[Point]


@dataclass
class PlannerTestCommand(PlannerCommand):
    areas: list[Area]
    obstacles: list[Obstacle]
    spline: Spline
    backward: bool = False


@dataclass
class PlannerObstacleDistanceCommand(PlannerCommand):
    areas: list[Area]
    obstacles: list[Obstacle]
    pose: Pose
    backward: bool = False


@dataclass
class PlannerResponse:
    id: str
    deadline: float
    content: Any


class PlannerProcess(Process):

    def __init__(self, connection: Connection, robot_outline: list[tuple[float, float]]):
        super().__init__()
        self.log = logging.getLogger('rosys.pathplanning.PlannerProcess')
        self.connection = connection
        self.planner = DelaunayPlanner(robot_outline)

    def run(self):
        icecream.install()  # NOTE provide ic(...) in subprocess
        while True:
            try:
                cmd = self.connection.recv()
            except EOFError:
                self.log.info('PlannerProcess stopped')
                return
            try:
                if isinstance(cmd, PlannerSearchCommand):
                    self.log.info(cmd)
                    additional_points = [cmd.start.point, cmd.goal.point]
                    self.planner.update_map(cmd.areas, cmd.obstacles, additional_points, cmd.deadline)
                    self.respond(cmd, self.planner.search(cmd.start, cmd.goal))
                if isinstance(cmd, PlannerGrowMapCommand):
                    self.planner.grow_map(cmd.points, cmd.deadline)
                    self.respond(cmd, None)
                if isinstance(cmd, PlannerTestCommand):
                    self.planner.update_map(cmd.areas, cmd.obstacles, [cmd.spline.start, cmd.spline.end], cmd.deadline)
                    self.respond(cmd, bool(self.planner.obstacle_map.test_spline(cmd.spline, cmd.backward)))
                if isinstance(cmd, PlannerObstacleDistanceCommand):
                    self.planner.update_map(cmd.areas, cmd.obstacles, [cmd.pose], cmd.deadline)
                    self.respond(cmd, self.planner.obstacle_map.get_distance(cmd.pose.x, cmd.pose.y, cmd.pose.yaw))
            except Exception as e:
                self.log.exception(f'failed to compute cmd "{cmd}"')
                self.respond(cmd, e)

    def respond(self, cmd: PlannerCommand, content: Any):
        self.connection.send(PlannerResponse(cmd.id, cmd.deadline, content))
