import rosys

from ..geometry import Velocity
from .robot_brain import RobotBrain
from .wheels import Wheels


class WheelsHardware(Wheels):
    '''This module implements wheels hardware.

    Drive and stop commands are forwarded to a given Robot Brain.
    Velocities are read and emitted regularly.
    '''

    def __init__(self, robot_brain: RobotBrain) -> None:
        super().__init__()

        self.robot_brain = robot_brain
        rosys.on_repeat(self.step, 0.01)
        rosys.on_shutdown(self.stop)

    async def drive(self, linear: float, angular: float) -> None:
        await self.robot_brain.send(f'wheels.speed({linear}, {angular})')

    async def stop(self) -> None:
        await self.robot_brain.send('wheels.off()')

    async def step(self) -> None:
        velocities: list[Velocity] = []
        for time, line in await self.robot_brain.read_lines():
            words = line.split()
            if words[0] == 'core':
                velocities.append(Velocity(linear=float(words[2]), angular=float(words[3]), time=time))
                await self.on_core_output(words[4:])
        self.VELOCITY_MEASURED.emit(velocities)

    async def on_core_output(self, words: list[str]) -> None:
        pass
