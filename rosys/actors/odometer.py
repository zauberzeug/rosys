import numpy as np
from .actor import Actor
from ..world.pose import PoseStep
from ..world.world import World
from ..helpers import angle


class Odometer(Actor):

    def __init__(self):
        super().__init__()

        self.last_time: float = None
        self.steps: list[PoseStep] = []
        self.flips: int = 0
        self.flip_detection_initialized: bool = False

    def handle_velocity(self, world: World):
        while any(world.robot.odometry):
            velocity = world.robot.odometry.pop(0)

            if self.last_time is None:
                self.last_time = velocity.time
                continue

            dt = velocity.time - self.last_time
            self.last_time = velocity.time

            step = PoseStep(linear=dt*velocity.linear, angular=dt*velocity.angular, time=world.time)
            self.steps.append(step)
            world.robot.prediction += step
            world.robot.simulation += step

            world.robot.current_velocity = velocity

            if step.linear or step.angular:
                world.robot.last_movement = step.time

        self.prune_steps(world.time - 10.0)

    def handle_detection(self, world: World):
        if world.robot.detection is None or not any(self.steps) or world.robot.detection.time < self.steps[0].time:
            return

        while self.steps[0].time < world.robot.detection.time:
            del self.steps[0]

        # NOTE: attempt to avoid 180-degree flips due to swapped marker points
        if self.flip_detection_initialized:
            dYaw = sum(step.angular for step in self.steps)
            if abs(angle(world.robot.prediction.yaw - dYaw, world.robot.detection.yaw)) > np.deg2rad(90):
                self.flips += 1
                for image in world.images:
                    if image.time == world.robot.detection.time:
                        self.log.warning(f'adding {image.id} to upload queue because our position has flipped')
                        world.upload.mark(image)
                if self.flips < 3:
                    self.log.warn('Avoiding flip')
                    return
            else:
                self.flips = 0

        world.robot.prediction = world.robot.detection.copy(deep=True)
        for step in self.steps:
            world.robot.prediction += step

        self.flip_detection_initialized = True

    def prune_steps(self, cut_off_time: float):
        while any(self.steps) and self.steps[0].time < cut_off_time:
            del self.steps[0]
