import abc
from dataclasses import dataclass, field
import icecream
import heapq
import logging
from multiprocessing import Process
from multiprocessing.connection import Connection
import numpy as np
import time
from typing import Any, Optional
import uuid

from .distance_map import DistanceMap
from .grid import Grid
from .obstacle_map import Obstacle, ObstacleMap
from .steps import Step
from ...helpers import angle
from ...world import Area, PathSegment, Point, Pose, Spline


@dataclass
class PlannerState:
    robot_outline: list[tuple[float, float]]
    obstacle_map: Optional[ObstacleMap] = None
    small_obstacle_map: Optional[ObstacleMap] = None
    distance_map: Optional[DistanceMap] = None
    obstacles: list[Obstacle] = field(default_factory=list)
    areas: list[Area] = field(default_factory=list)
    goal: Optional[Pose] = None


@dataclass
class PlannerCommand(abc.ABC):
    deadline: float

    def __post_init__(self):
        self.id = str(uuid.uuid4())


@dataclass
class PlannerGetStateCommand(PlannerCommand):
    pass


@dataclass
class PlannerGrowMapCommand(PlannerCommand):
    points: list[Point]


@dataclass
class PlannerSearchCommand(PlannerCommand):
    areas: list[Area]
    obstacles: list[Obstacle]
    start: Pose
    goal: Pose
    backward: bool


@dataclass
class PlannerTestCommand(PlannerCommand):
    areas: list[Area]
    obstacles: list[Obstacle]
    spline: Spline
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
        self.state = PlannerState(robot_outline)

    def run(self):
        icecream.install()  # NOTE provide ic(...) in subprocess
        while True:
            try:
                cmd = self.connection.recv()
            except EOFError:
                self.log.info('PlannerProcess stopped')
                return
            try:
                if isinstance(cmd, PlannerGetStateCommand):
                    self.respond(cmd, self.state)
                if isinstance(cmd, PlannerGrowMapCommand):
                    self.grow_obstacle_map(cmd.points, cmd.deadline)
                    self.respond(cmd, None)
                if isinstance(cmd, PlannerSearchCommand):
                    additional_points = [cmd.start.point, cmd.goal.point]
                    self.update_obstacle_map(cmd.areas, cmd.obstacles, additional_points, cmd.deadline)
                    self.update_distance_map(cmd.goal, cmd.deadline)
                    self.respond(cmd, self.search(cmd.goal, cmd.start, cmd.backward, cmd.deadline))
                if isinstance(cmd, PlannerTestCommand):
                    self.update_obstacle_map(cmd.areas, cmd.obstacles, [cmd.spline.start, cmd.spline.end], cmd.deadline)
                    self.respond(cmd, bool(self.state.obstacle_map.test_spline(cmd.spline, cmd.backward)))
            except Exception as e:
                self.log.exception(f'failed to compute cmd "{cmd}"')
                self.respond(cmd, e)

    def respond(self, cmd: PlannerCommand, content: Any):
        self.connection.send(PlannerResponse(cmd.id, cmd.deadline, content))

    def recreate_obstacle_map(self,
                              areas: list[Area],
                              obstacles: list[Obstacle],
                              additional_points: list[Point],
                              deadline: float):
        points = [p for obstacle in obstacles for p in obstacle.outline]
        points += [p for area in areas for p in area.outline]
        points += additional_points
        g1 = Grid.from_points(points, 0.1, 36, padding=1.0)
        g2 = Grid.from_points(points, 0.2, 0, padding=1.0)
        self.state.obstacle_map = ObstacleMap.from_world(self.state.robot_outline, areas, obstacles, g1, deadline)
        self.state.small_obstacle_map = ObstacleMap.from_world(self.state.robot_outline, areas, obstacles, g2, deadline)

    def update_obstacle_map(self,
                            areas: list[Area],
                            obstacles: list[Obstacle],
                            additional_points: list[Point],
                            deadline: float):
        if self.state.obstacle_map and \
                self.state.areas == areas and \
                self.state.obstacles == obstacles and \
                all(self.state.obstacle_map.grid.contains(point, padding=1.0) for point in additional_points):
            return
        self.recreate_obstacle_map(areas, obstacles, additional_points, deadline)
        self.state.distance_map = None
        self.state.areas = areas
        self.state.obstacles = obstacles

    def grow_obstacle_map(self, points: list[Point], deadline: float):
        if self.state.obstacle_map is not None and \
                all(self.state.obstacle_map.grid.contains(point, padding=1.0) for point in points):
            return
        if self.state.obstacle_map is not None:
            bbox = self.state.obstacle_map.grid.bbox
            points.append(Point(x=bbox[0],         y=bbox[1]))
            points.append(Point(x=bbox[0]+bbox[2], y=bbox[1]))
            points.append(Point(x=bbox[0],         y=bbox[1]+bbox[3]))
            points.append(Point(x=bbox[0]+bbox[2], y=bbox[1]+bbox[3]))
        self.recreate_obstacle_map(self.state.areas, self.state.obstacles, points, deadline)
        self.state.distance_map = None

    def update_distance_map(self, goal: Pose, deadline: float) -> None:
        if self.state.distance_map and self.state.goal == goal:
            return
        self.state.distance_map = DistanceMap(self.state.small_obstacle_map, goal, deadline)
        self.state.goal = goal

    def search(self, goal: Pose, start: Pose, backward: bool, deadline: float) -> Optional[list[PathSegment]]:
        step_dist = 0.5
        num_candidates = 16

        pose = (start.x, start.y, start.yaw)
        heap = [(self.state.distance_map.interpolate(start.x, start.y), 0, Step(pose))]
        visited = set()

        if self.state.obstacle_map.test(goal.x, goal.y, goal.yaw):
            raise RuntimeError('target pose is unreachable')

        while pose != (goal.x, goal.y, goal.yaw):
            if time.time() > deadline:
                raise TimeoutError('could not find path')

            try:
                _, travel_cost, step = heapq.heappop(heap)
            except IndexError:
                raise RuntimeError('could not find path')

            pose = step.target
            tup = tuple(np.round(pose, 3))
            if tup in visited:
                continue
            visited.add(tup)

            if pose == (goal.x, goal.y, goal.yaw):
                break

            goal_dist = np.sqrt((goal.x - pose[0])**2 + (goal.y - pose[1])**2)
            goal_candidate = [(goal.x, goal.y, goal.yaw)] if goal_dist < 3 * step_dist else []
            fw_candidates = [
                (pose[0] + step_dist * np.cos(yaw), pose[1] + step_dist * np.sin(yaw), yaw)
                for yaw in np.linspace(-np.pi / 2, np.pi / 2, num_candidates // 2 + 1)[1:-1] + pose[2]
            ]
            bw_candidates = [
                (pose[0] - step_dist * np.cos(yaw), pose[1] - step_dist * np.sin(yaw), yaw)
                for yaw in np.linspace(-np.pi / 2, np.pi / 2, num_candidates // 2 + 1)[1:-1] + pose[2]
            ]

            for candidate in goal_candidate + fw_candidates + bw_candidates:
                tup = tuple(np.round(candidate, 3))
                if tup in visited:
                    continue

                if self.state.obstacle_map.test(*candidate):
                    continue

                costs = None
                for backward in [False, True]:
                    if backward and candidate in fw_candidates:
                        continue
                    if not backward and candidate in bw_candidates:
                        continue

                    next_step = Step(candidate, step, backward=backward)
                    if candidate in goal_candidate:
                        if not next_step.is_healthy():
                            continue

                    if self.state.obstacle_map.get_minimum_spline_distance(next_step.spline, backward=backward) < 0.1:
                        continue

                    if costs is None:
                        dist = self.state.distance_map.interpolate(candidate[0], candidate[1])[0]
                        new_travel_cost = travel_cost + step_dist
                        if step.backward != next_step.backward:
                            new_travel_cost += step_dist
                        yaw_cost = 0
                        if dist > step_dist:
                            gx, gy = self.state.distance_map.gradient(candidate[0], candidate[1])
                            target_yaw = np.arctan2(gy, gx) if backward else np.arctan2(-gy, -gx)
                            yaw_error = abs(angle(candidate[2], target_yaw[0]))
                            if yaw_error > np.pi / 2:
                                yaw_cost = 5 * yaw_error
                        obstacle_cost = max(1.0 - self.state.obstacle_map.get_distance(*candidate), 0.0)
                        remaining_cost = dist + 0.7 * new_travel_cost + yaw_cost + 0.3 * obstacle_cost
                        costs = (remaining_cost, new_travel_cost)

                    heapq.heappush(heap, (costs[0], costs[1], next_step))

        path = step.backtrace()
        path.smooth(self.state.obstacle_map, control_dist=0.5)
        return [PathSegment(spline=step.spline, backward=step.backward) for step in path[1:]]
