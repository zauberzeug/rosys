from .actor import Actor
from ..world.pose import PoseStep
from ..world.world import World


class Odometer(Actor):

    def __init__(self):

        self.last_time: float = None
        self.steps = []

    def handle_velocity(self, world: World):

        if self.last_time is None:
            self.last_time = world.time
            return

        dt = world.time - self.last_time
        self.last_time = world.time

        robot = world.robot
        step = PoseStep(
            linear=dt * robot.velocity.linear,
            angular=dt * robot.velocity.angular,
            time=world.time,
        )
        self.steps.append(step)
        self.prune_steps(world.time - 10.0)

        robot.prediction += step

    def handle_camera_pose(self, world: World):

        ...

    def prune_steps(self, cut_off_time: float):

        while any(self.steps) and self.steps[0].time < cut_off_time:
            del self.steps[0]
