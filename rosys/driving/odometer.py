import logging
from copy import deepcopy
from typing import Protocol

from .. import rosys
from ..event import Event
from ..geometry import Pose, Pose3d, PoseStep, Rotation, Velocity


class VelocityProvider(Protocol):
    VELOCITY_MEASURED: Event


class Odometer:
    """An odometer collects velocity information from a given wheels module (or any velocity-providing hardware representation).

    It can also handle "detections", i.e. absolute pose information with timestamps.
    Given the history of previously received velocities, it can update its prediction of the current pose.

    The `get_pose` method provides robot poses from the within the last 10 seconds.
    """

    def __init__(self, wheels: VelocityProvider) -> None:
        self.WHEELS_TURNED = Event[[]]()
        """the wheels have turned with non-zero velocity"""

        self.PREDICTION_UPDATED = Event[[]]()
        """the pose prediction has been updated"""

        self.log = logging.getLogger('rosys.odometer')

        wheels.VELOCITY_MEASURED.register(self.handle_velocities)
        self.prediction: Pose = Pose()
        self.prediction_frame = Pose3d().as_frame('rosys.odometer.prediction')
        self.detection: Pose | None = None
        self.current_velocity: Velocity | None = None
        self.last_movement: float = 0
        self.history: list[Pose] = []
        self.odometry_frame: Pose = Pose()

        rosys.on_repeat(self.prune_history, 1.0)

    def handle_velocities(self, velocities: list[Velocity]) -> None:
        robot_moved: bool = False
        for velocity in velocities:
            if not self.history:
                self.history.append(Pose(time=velocity.time))
                continue

            dt = velocity.time - self.history[-1].time
            step = PoseStep(linear=dt*velocity.linear, angular=dt*velocity.angular, time=velocity.time)
            self.history.append(self.history[-1] + step)

            if step.linear or step.angular:
                robot_moved = True

        if self.history:
            self.prediction = self.odometry_frame.transform_pose(self.history[-1])
        if velocities:
            self.current_velocity = velocities[-1]
        if robot_moved:
            self.last_movement = step.time
            self._handle_movement()
            self.WHEELS_TURNED.emit()

    def handle_detection(self, detection: Pose) -> None:
        self.detection = detection

        if self.history:
            self.odometry_frame = self._compute_odometry_frame(self.get_pose(detection.time, local=True), detection)
            self.prediction = self.odometry_frame.transform_pose(self.history[-1])
        else:
            self.prediction = deepcopy(detection)
        self._handle_movement()

    def _handle_movement(self) -> None:
        self.prediction_frame.x = self.prediction.x
        self.prediction_frame.y = self.prediction.y
        self.prediction_frame.rotation = Rotation.from_euler(0, 0, self.prediction.yaw)
        self.PREDICTION_UPDATED.emit()

    def prune_history(self, max_age: float = 10.0) -> None:
        cut_off_time = rosys.time() - max_age
        while self.history and self.history[0].time <= cut_off_time:
            del self.history[0]

    def get_pose(self, time: float, local: bool = False) -> Pose:
        for i in range(1, len(self.history)):
            if self.history[i-1].time <= time < self.history[i].time:
                f = (time - self.history[i-1].time) / (self.history[i].time - self.history[i-1].time)
                local_pose = self.history[i-1].interpolate(self.history[i], f)
                return local_pose if local else self.odometry_frame.transform_pose(local_pose)
        if local:
            return deepcopy(self.history[-1])
        return Pose(x=self.prediction.x, y=self.prediction.y, yaw=self.prediction.yaw, time=time)

    @staticmethod
    def _compute_odometry_frame(local_pose: Pose, global_pose: Pose) -> Pose:
        frame = Pose.from_matrix(global_pose.matrix @ local_pose.inv_matrix)
        frame.time = global_pose.time
        return frame

    def reset(self) -> None:
        self.prediction = Pose()
        self.detection = None
        self.current_velocity = None
        self.last_movement = 0
        self.history.clear()
        self.odometry_frame = Pose()
