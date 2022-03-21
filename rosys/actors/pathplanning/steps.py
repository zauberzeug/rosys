from __future__ import annotations
from typing import Optional
import numpy as np
from ...world import Pose, Spline
from ...helpers import angle


class Path(list):

    @staticmethod
    def from_poses(poses):
        steps = [Step(poses[0])]
        for pose in poses[1:]:
            steps += [Step(pose, steps[-1])]
        return Path(steps)

    def smooth(self, obstacle_map, control_dist=None):
        while True:
            for s in range(1, len(self) - 1):
                if self[s].backward != self[s+1].backward:
                    continue

                new_step = Step(self[s+1].target, self[s].previous_step,
                                control_dist=control_dist, backward=self[s].backward)
                if not new_step.is_healthy():
                    continue
                new_distance = obstacle_map.get_minimum_spline_distance(new_step.spline, backward=new_step.backward)
                if new_distance == 0:
                    continue
                old_distance1 = obstacle_map.get_minimum_spline_distance(self[s].spline, self[s].backward)
                old_distance2 = obstacle_map.get_minimum_spline_distance(self[s+1].spline, self[s+1].backward)
                if new_distance < min(old_distance1, old_distance2):
                    continue
                self[s] = new_step
                del self[s+1]
                break  # restart while loop
            else:
                break  # exit while loop


class Step:

    def __init__(self,
                 target: tuple[float, float, float],
                 previous_step: Optional[Step] = None,
                 control_dist: Optional[float] = None,
                 backward: bool = False):
        self.previous_step = previous_step
        self.target = target
        self.backward = backward

        start = previous_step.target if previous_step is not None else target
        self.spline = Spline.from_poses(
            Pose(x=start[0], y=start[1], yaw=start[2]),
            Pose(x=target[0], y=target[1], yaw=target[2]),
            control_dist=control_dist, backward=backward)

    def __lt__(self, other):
        return self.target < other.target

    def is_healthy(self, curvature_limit=10.0):
        dir_ = self.spline.start.direction(self.spline.end)
        yaw0 = self.spline.start.direction(self.spline.control1)
        yaw1 = self.spline.control2.direction(self.spline.end)
        return \
            np.abs(angle(dir_, yaw0)) < np.pi / 2 and \
            np.abs(angle(dir_, yaw1)) < np.pi / 2 and \
            np.abs(self.spline.max_curvature()) < curvature_limit  # NOTE: max_curvature can be NaN

    def backtrace(self):
        steps = [self]
        while steps[0].previous_step is not None:
            steps.insert(0, steps[0].previous_step)
        return Path(steps)
