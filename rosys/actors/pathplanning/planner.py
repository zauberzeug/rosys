import numpy as np
import heapq
import copy
import time
from rosys.helpers import angle
from rosys.actors.pathplanning.distance_map import DistanceMap
from rosys.actors.pathplanning.steps import Step


class Planner:

    def __init__(self, obstacle_map, small_obstacle_map=None, timeout=None):
        self.obstacle_map = obstacle_map
        self.small_obstacle_map = small_obstacle_map or obstacle_map
        self.timeout = timeout

    def set_goal(self, goal, backward_to_goal=False):
        self.goal = goal
        self.backward_to_goal = backward_to_goal
        self.distance_map = DistanceMap(self.small_obstacle_map, goal)

    def search(self, pose):
        start_time = time.time()

        step_dist = 0.5
        num_candidates = 16

        heap = [(self.distance_map.interpolate(pose[0], pose[1]), 0, Step(pose))]
        visited = set()

        while pose != self.goal:
            if self.timeout is not None and time.time() - start_time > self.timeout:
                raise TimeoutError("Could not find a path")

            try:
                _, travel_cost, step = heapq.heappop(heap)
            except IndexError:
                raise TimeoutError("Could not find a path")

            pose = step.target
            tup = tuple(np.round(pose, 3))
            if tup in visited:
                continue
            visited.add(tup)

            if pose == self.goal:
                break

            goal_dist = np.sqrt((self.goal[0] - pose[0])**2 + (self.goal[1] - pose[1])**2)
            goal_candidate = [self.goal] if goal_dist < 3 * step_dist else []
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
                            target_yaw = np.arctan2(gy, gx) if self.backward_to_goal else np.arctan2(-gy, -gx)
                            yaw_error = np.abs(angle(candidate[2], target_yaw))
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
