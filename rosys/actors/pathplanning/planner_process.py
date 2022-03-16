import abc
from dataclasses import dataclass
import icecream
import heapq
import logging
from multiprocessing import Process
from multiprocessing.connection import Connection
import numpy as np
import time
from typing import Optional

from .distance_map import DistanceMap
from .grid import Grid
from .obstacle_map import Obstacle, ObstacleMap
from .steps import Step
from ...helpers import angle
from ...world import Area, PathSegment, Point, Pose, Spline


@dataclass
class PlannerCommand(abc.ABC):
    timeout: float
    areas: list[Area]
    obstacles: list[Obstacle]


@dataclass
class PlannerSearchCommand(PlannerCommand):
    start: Pose
    goal: Pose
    backward: bool


@dataclass
class PlannerTestCommand(PlannerCommand):
    spline: Spline
    backward: bool = False


class PlannerProcess(Process):

    def __init__(self, connection: Connection, robot_outline: list[tuple[float, float]]):
        super().__init__()
        self.log = logging.getLogger('rosys.pathplanning.PlannerProcess')
        self.connection = connection
        self.obstacle_map: Optional[ObstacleMap] = None
        self.small_obstacle_map: Optional[ObstacleMap] = None
        self.distance_map: Optional[DistanceMap] = None
        self.robot_outline = robot_outline
        self.obstacles: list[Obstacle] = []
        self.areas: list[Area] = []
        self.goal: Optional[Pose] = None

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
                    try:
                        self.update_obstacle_map(cmd.areas, cmd.obstacles, [cmd.start.point, cmd.goal.point])
                        self.update_distance_map(cmd.goal)
                        self.connection.send(self.search(cmd.goal, cmd.start, cmd.backward, cmd.timeout))
                    except (TimeoutError, RuntimeError) as e:
                        self.connection.send(e)
                if isinstance(cmd, PlannerTestCommand):
                    self.update_obstacle_map(cmd.areas, cmd.obstacles, [cmd.spline.start, cmd.spline.end])
                    self.connection.send(self.obstacle_map.test_spline(cmd.spline, cmd.backward))
            except Exception as e:
                self.log.exception(f'failed to compute cmd "{cmd}"')
                self.connection.send(e)

    def update_obstacle_map(self, areas: list[Area], obstacles: list[Obstacle], more_points: list[Point] = []) -> None:
        if self.obstacle_map and \
                self.areas == areas and \
                self.obstacles == obstacles and \
                all(self.obstacle_map.grid.contains(point, padding=1.0) for point in more_points):
            return
        points = [p for obstacle in self.obstacles for p in obstacle.outline]
        points += [p for area in self.areas for p in area.outline]
        grid = Grid.from_points(points + more_points, 0.1, 36, padding=1.0)
        self.obstacle_map = ObstacleMap.from_world(self.robot_outline, areas, obstacles, grid)
        self.small_obstacle_map = self.obstacle_map  # TODO?
        self.distance_map = None
        self.areas = areas
        self.obstacles = obstacles

    def update_distance_map(self, goal: Pose) -> None:
        if self.distance_map and self.goal == goal:
            return
        self.distance_map = DistanceMap(self.small_obstacle_map, goal)
        self.goal = goal

    def search(self, goal: Pose, start: Pose, backward: bool, timeout: float) -> Optional[list[PathSegment]]:
        start_time = time.time()
        step_dist = 0.5
        num_candidates = 16

        pose = (start.x, start.y, start.yaw)
        heap = [(self.distance_map.interpolate(start.x, start.y), 0, Step(pose))]
        visited = set()

        while pose != (goal.x, goal.y, goal.yaw):
            if timeout is not None and time.time() - start_time > timeout:
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

                if self.obstacle_map.test(*candidate):
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

                    if self.obstacle_map.get_minimum_spline_distance(next_step.spline, backward=backward) < 0.1:
                        continue

                    if costs is None:
                        dist = self.distance_map.interpolate(candidate[0], candidate[1])[0]
                        new_travel_cost = travel_cost + step_dist
                        if step.backward != next_step.backward:
                            new_travel_cost += step_dist
                        yaw_cost = 0
                        if dist > step_dist:
                            gx, gy = self.distance_map.gradient(candidate[0], candidate[1])
                            target_yaw = np.arctan2(gy, gx) if backward else np.arctan2(-gy, -gx)
                            yaw_error = abs(angle(candidate[2], target_yaw[0]))
                            if yaw_error > np.pi / 2:
                                yaw_cost = 5 * yaw_error
                        obstacle_cost = max(1.0 - self.obstacle_map.get_distance(*candidate), 0.0)
                        remaining_cost = dist + 0.7 * new_travel_cost + yaw_cost + 0.3 * obstacle_cost
                        costs = (remaining_cost, new_travel_cost)

                    heapq.heappush(heap, (costs[0], costs[1], next_step))

        path = step.backtrace()
        path.smooth(self.obstacle_map, control_dist=0.5)
        return [PathSegment(spline=step.spline, backward=step.backward) for step in path[1:]]
