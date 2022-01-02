import numpy as np
import heapq
import copy
import time
from ..helpers import angle
from ..world.pose import Pose
from ..world.world import World
from .distance_map import DistanceMap
from .grid import Grid
from .obstacle_map import ObstacleMap
from .steps import Step


class Planner:

    def __init__(self, world: World):
        self.world = world
        self.obstacle_map = None
        self.small_obstacle_map = None
        self.obstacles = None
        self.goal = None

    def search(self, *, goal: Pose, start: Pose = None, backward: bool = False, timeout: float = None):
        if start is None:
            start = self.world.robot.prediction

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

        while pose != [goal.x, goal.y, goal.yaw]:
            if timeout is not None and time.time() - start_time > timeout:
                raise TimeoutError('Could not find a path')

            try:
                _, travel_cost, step = heapq.heappop(heap)
            except IndexError:
                raise TimeoutError('Could not find a path')

            pose = step.target
            tup = tuple(np.round(pose, 3))
            if tup in visited:
                continue
            visited.add(tup)

            if pose == [goal.x, goal.y, goal.yaw]:
                break

            goal_dist = np.sqrt((goal.x - pose[0])**2 + (goal.y - pose[1])**2)
            goal_candidate = [[goal.x, goal.y, goal.yaw]] if goal_dist < 3 * step_dist else []
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

        self.raw_path = step.backtrace()
        self.path = copy.deepcopy(self.raw_path)
        self.path.smooth(self.obstacle_map, control_dist=0.5)
        del self.path[0]

    def update_obstacle_map(self, goal: Pose, start: Pose) -> None:
        if self.obstacles != self.world.obstacles or \
                not self.obstacle_map.grid.contains(start.point, padding=1.0) or \
                not self.obstacle_map.grid.contains(goal.point, padding=1.0):
            points = [p for obstacle in self.world.obstacles.values() for p in obstacle.outline]
            grid = Grid.from_points(points + [start.point, goal.point], 0.1, 36, padding=1.0)
            self.obstacle_map = ObstacleMap.from_world(self.world, grid)
            self.small_obstacle_map = self.obstacle_map  # TODO?
            self.obstacles = copy.deepcopy(self.world.obstacles)
            self.goal = None
