import logging
from typing import Optional

from .. import event
from ..runtime import runtime
from ..world import Pose, PoseStep, Velocity


class Odometer:

    def __init__(self):
        self.log = logging.getLogger(self.__class__.__name__)

        self.odometry: list[Velocity] = []
        self.prediction: Pose = Pose()
        self.detection: Optional[Pose] = None
        self.current_velocity: Optional[Velocity] = None
        self.last_movement: float = 0

        self._last_time: float = None
        self._steps: list[PoseStep] = []

    def add_odometry(self, linear_velocity: float, angular_velocity: float, time: float) -> None:
        self.odometry.append(Velocity(linear_velocity, angular_velocity, time))

    def process_odometry(self) -> None:
        if not self.odometry:
            return
        while self.odometry:
            velocity = self.odometry.pop(0)

            if self._last_time is None:
                self._last_time = velocity.time
                continue

            dt = velocity.time - self._last_time
            self._last_time = velocity.time

            step = PoseStep(linear=dt*velocity.linear, angular=dt*velocity.angular, time=runtime.time)
            self._steps.append(step)
            self.prediction += step

            self.current_velocity = velocity

            if step.linear or step.angular:
                self.last_movement = step.time
                event.emit(event.Id.ROBOT_MOVED)

        self.prune_steps(runtime.time - 10.0)

    def process_detection(self) -> None:
        if self.detection is None:
            return

        if self._steps and self.detection.time < self._steps[0].time:
            return

        self.prune_steps(self.detection.time)
        self.prediction = self.detection.copy(deep=True)
        for step in self._steps:
            self.prediction += step

    def prune_steps(self, cut_off_time: float) -> None:
        while self._steps and self._steps[0].time < cut_off_time:
            del self._steps[0]
