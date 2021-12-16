import numpy as np
from .. import event
from ..helpers import angle
from ..world import PoseStep
from .actor import Actor


class Odometer(Actor):

    def __init__(self):
        super().__init__()

        self.last_time: float = None
        self.steps: list[PoseStep] = []
        self.flips: int = 0
        self.flip_detection_initialized: bool = False
        event.register(event.Id.NEW_MACHINE_DATA, self.handle_velocity)

    def handle_velocity(self):
        if not self.world.robot.odometry:
            return
        while self.world.robot.odometry:
            velocity = self.world.robot.odometry.pop(0)

            if self.last_time is None:
                self.last_time = velocity.time
                continue

            dt = velocity.time - self.last_time
            self.last_time = velocity.time

            step = PoseStep(linear=dt*velocity.linear, angular=dt*velocity.angular, time=self.world.time)
            self.steps.append(step)
            self.world.robot.prediction += step
            self.world.robot.simulation += step

            self.world.robot.current_velocity = velocity

            if step.linear or step.angular:
                self.world.robot.last_movement = step.time
                event.emit(event.Id.ROBOT_MOVED)

        self.prune_steps(self.world.time - 10.0)

    def handle_detection(self):
        if self.world.robot.detection is None or not any(self.steps) or self.world.robot.detection.time < self.steps[0].time:
            return

        while self.steps[0].time < self.world.robot.detection.time:
            del self.steps[0]

        # NOTE: attempt to avoid 180-degree flips due to swapped marker points
        if self.flip_detection_initialized:
            dYaw = sum(step.angular for step in self.steps)
            if abs(angle(self.world.robot.prediction.yaw - dYaw, self.world.robot.detection.yaw)) > np.deg2rad(90):
                self.flips += 1
                for image in self.world.images:
                    if image.time == self.world.robot.detection.time:
                        self.log.warning(f'adding {image.id} to upload queue because our position has flipped')
                        self.world.upload.mark(image)
                if self.flips < 3:
                    self.log.warn('Avoiding flip')
                    return
            else:
                self.flips = 0

        self.world.robot.prediction = self.world.robot.detection.copy(deep=True)
        for step in self.steps:
            self.world.robot.prediction += step

        self.flip_detection_initialized = True

    def prune_steps(self, cut_off_time: float):
        while any(self.steps) and self.steps[0].time < cut_off_time:
            del self.steps[0]
