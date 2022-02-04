from .. import event
from ..world import PoseStep
from .actor import Actor


class Odometer(Actor):

    def __init__(self):
        super().__init__()

        self.last_time: float = None
        self.steps: list[PoseStep] = []
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
        if self.world.robot.detection is None:
            return

        if self.steps and self.world.robot.detection.time < self.steps[0].time:
            return

        self.prune_steps(self.world.robot.detection.time)
        self.world.robot.prediction = self.world.robot.detection.copy(deep=True)
        for step in self.steps:
            self.world.robot.prediction += step

    def prune_steps(self, cut_off_time: float):
        while self.steps and self.steps[0].time < cut_off_time:
            del self.steps[0]
