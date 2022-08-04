import logging
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional, Protocol

import rosys

from ..event import Event
from ..geometry import Pose, PoseStep, Velocity


@dataclass(slots=True, kw_only=True)
class HistoryItem:
    pose: Pose
    step: PoseStep


class VelocityProvider(Protocol):
    VELOCITY_MEASURED: Event


class Odometer:
    ROBOT_MOVED = Event()
    '''a robot movement is detected'''

    def __init__(self, wheels: VelocityProvider) -> None:
        self.log = logging.getLogger('rosys.odometer')

        wheels.VELOCITY_MEASURED.register(self.handle_velocities)
        self.prediction: Pose = Pose()
        self.detection: Optional[Pose] = None
        self.current_velocity: Optional[Velocity] = None
        self.last_movement: float = 0

        self._last_time: float = None
        self._history: list[HistoryItem] = []

        rosys.on_repeat(lambda: self.prune_history(rosys.time() - 10.0), 1.0)

    def handle_velocities(self, velocities: list[Velocity]) -> None:
        for velocity in velocities:
            if self._last_time is None:
                self._last_time = velocity.time
                continue

            dt = velocity.time - self._last_time
            self._last_time = velocity.time

            step = PoseStep(linear=dt*velocity.linear, angular=dt*velocity.angular, time=velocity.time, dt=dt)
            self._history.append(HistoryItem(pose=deepcopy(self.prediction), step=step))
            self.prediction += step

            self.current_velocity = velocity

            if step.linear or step.angular:
                self.last_movement = step.time
                self.ROBOT_MOVED.emit()

    def handle_detection(self, detection: Pose) -> None:
        self.detection = detection
        self.prediction = deepcopy(detection)

        self.prune_history(detection.time)
        if not self._history:
            return

        step0 = self._history[0].step
        dt = step0.time - detection.time
        fraction = dt / step0.dt
        half_step = PoseStep(linear=fraction*step0.linear, angular=fraction*step0.angular, time=step0.time, dt=dt)
        self.prediction += half_step
        self._history[0].pose = deepcopy(detection)
        self._history[0].step = half_step
        for i in range(1, len(self._history)):
            self._history[i].pose = self._history[i-1].pose + self._history[i-1].step
            self.prediction += self._history[i].step

    def prune_history(self, cut_off_time: float) -> None:
        while self._history and self._history[0].step.time <= cut_off_time:
            del self._history[0]

    def get_pose(self, time: float) -> Pose:
        for item in self._history:
            if item.pose.time <= time < item.step.time:
                f = (time - item.pose.time) / (item.step.time - item.pose.time)
                return item.pose + PoseStep(linear=f*item.step.linear, angular=f*item.step.angular, time=time)
        return Pose(x=self.prediction.x, y=self.prediction.y, yaw=self.prediction.yaw, time=time)
