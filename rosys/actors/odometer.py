import logging
from copy import deepcopy
from typing import Optional

from ..event import Event
from ..runtime import runtime
from ..world import Pose, PoseStep, Velocity


class Odometer:
    ROBOT_MOVED = Event()
    '''a robot movement is detected'''

    def __init__(self):
        self.log = logging.getLogger('rosys.odometer')

        self.odometry: list[Velocity] = []
        self.prediction: Pose = Pose()
        self.detection: Optional[Pose] = None
        self.current_velocity: Optional[Velocity] = None
        self.last_movement: float = 0

        self._last_time: float = None
        self._steps: list[PoseStep] = []

    def add_odometry(self, linear_velocity: float, angular_velocity: float, time: float) -> None:
        self.odometry.append(Velocity(linear=linear_velocity, angular=angular_velocity, time=time))

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
                self.ROBOT_MOVED.emit()

        self.prune_steps(runtime.time - 10.0)

    def handle_detection(self, detection: Pose) -> None:
        self.detection = detection

        if self._steps and detection.time < self._steps[0].time:
            return

        self.prune_steps(detection.time)
        self.prediction = deepcopy(detection)
        for step in self._steps:
            self.prediction += step

    def prune_steps(self, cut_off_time: float) -> None:
        while self._steps and self._steps[0].time < cut_off_time:
            del self._steps[0]
