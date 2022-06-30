from ..actors.odometer import Odometer
from ..lifecycle import on_repeat, on_shutdown
from .robot_brain import RobotBrain
from .wheels import Wheels


class WheelsHardware(Wheels):

    def __init__(self, odometer: Odometer, robot_brain: RobotBrain) -> None:
        super().__init__(odometer)

        self.robot_brain = robot_brain

        on_shutdown(lambda: self.drive(0, 0))
        on_repeat(self.step, 0.01)

    async def drive(self, linear: float, angular: float) -> None:
        await self.robot_brain.send(f'wheels.speed({linear}, {angular})')

    async def stop(self) -> None:
        await self.robot_brain.send('wheels.off()')

    async def step(self) -> None:
        for time, line in await self.robot_brain.read_lines():
            words = line.split()
            if words.pop(0) == 'core':
                self.odometer.add_odometry(float(words.pop(0)), float(words.pop(0)), time)
        self.odometer.process_odometry()
