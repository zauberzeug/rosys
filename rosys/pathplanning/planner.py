import logging
from typing import Any
import numpy as np
import heapq
import copy
import time
import functools
from rosys.world.area import Area

from rosys.world.obstacle import Obstacle
from .. import run
from ..helpers import angle
from ..world import Pose, PathSegment, World
from .distance_map import DistanceMap
from .grid import Grid
from .obstacle_map import ObstacleMap
from .steps import Path, Step
from multiprocessing import Process, Pipe
from multiprocessing.connection import Connection
import asyncio


class Planner:

    def __init__(self, world: World):
        self.world = world
        self.connection, process_connection = Pipe()
        self.process = PlannerProcess(process_connection, self.world.robot.shape.outline)
        self.process.start()

    async def search_async(self, *, goal: Pose, start: Pose = None, backward: bool = False, timeout: float = 3.0):
        if start is None:
            start = self.world.robot.prediction
        await self.ensure_map()
        result = await self.call(['search', goal, start, backward], timeout)
        if result is None:
            raise TimeoutError('Could not find a path')
        else:
            return result

    async def call(self, cmd: list, timeout: float) -> Any:
        self.connection.send(cmd)
        t = time.time()
        while not self.connection.poll():
            if time.time() - t > timeout:
                raise TimeoutError('process call took too long')
            await asyncio.sleep(0.1)
        return self.connection.recv()

    async def ensure_map(self):
        pass
        # hash(frozenset(my_dict.items()))


class PlannerProcess(Process):

    def __init__(self, connection: Connection, robot_outline: list[tuple[float, float]]):
        super().__init__()
        self.log = logging.getLogger('rosys.path_planning.PlannerProcess')
        self.connection = connection
        self.obstacle_map = None
        self.small_obstacle_map = None
        self.robot_outline = robot_outline
        self.obstacles: list[Obstacle] = []
        self.areas: list[Area] = []
        self.goal = None

    def run(self):
        while True:
            try:
                cmd = self.connection.recv()
            except EOFError:
                self.log.info('PlannerProcess stopped')
                return
            try:
                if cmd[0] == 'search':
                    try:
                        self.connection.send(self.search(*cmd[1:]))
                    except:
                        self.connection.send(None)
            except:
                self.log.exception(f'failed to compute cmd "{cmd}"')

    def update_obstacle_map(self, goal: Pose, start: Pose) -> None:
        if self.obstacle_map and all([self.obstacle_map.grid.contains(pose.point, padding=1.0) for pose in [goal, start]]):
            return
        points = [p for obstacle in self.obstacles for p in obstacle.outline]
        points += [p for area in self.areas for p in area.outline]
        grid = Grid.from_points(points + [start.point, goal.point], 0.1, 36, padding=1.0)
        self.obstacle_map = ObstacleMap.from_world(self.robot_outline, self.areas, self.obstacles, grid)
        self.small_obstacle_map = self.obstacle_map  # TODO?
        self.goal = None

    def search(self, goal: Pose, start: Pose = None, backward: bool = False, timeout: float = None) -> list[PathSegment]:
        self.update_obstacle_map(goal, start)
        if self.goal != goal:
            self.distance_map = DistanceMap(self.small_obstacle_map, goal)
            self.goal = copy.deepcopy(goal)

        start_time = time.time()
        step_dist = 0.5
        num_candidates = 16

        pose = (start.x, start.y, start.yaw)
        heap = [(self.distance_map.interpolate(start.x, start.y), 0, Step(pose))]
        visited = set()

        while pose != (goal.x, goal.y, goal.yaw):
            if timeout is not None and time.time() - start_time > timeout:
                return None

            try:
                _, travel_cost, step = heapq.heappop(heap)
            except IndexError:
                return None

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

        self.raw_path: Path = step.backtrace()
        self.path: Path = copy.deepcopy(self.raw_path)
        self.path.smooth(self.obstacle_map, control_dist=0.5)
        del self.path[0]
        return [PathSegment(spline=step.spline, backward=step.backward) for step in self.path]
