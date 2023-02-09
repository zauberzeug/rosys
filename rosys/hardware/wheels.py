import abc

from .. import rosys
from ..event import Event
from ..geometry import Pose, PoseStep, Velocity
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Wheels(Module):
    '''The wheels module is a simple example for a representation of real or simulated robot hardware.

    Wheels can be moved using the `drive` methods and provide measured velocities as an event.
    '''

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.VELOCITY_MEASURED = Event()
        '''new velocity measurements are available for processing (argument: list of velocities)'''

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        pass

    @abc.abstractmethod
    async def stop(self) -> None:
        pass


class WheelsHardware(Wheels, ModuleHardware):
    '''This module implements wheels hardware.

    Drive and stop commands are forwarded to a given Robot Brain.
    Velocities are read and emitted regularly.
    '''

    def __init__(self, robot_brain: RobotBrain) -> None:
        super().__init__(robot_brain=robot_brain)

    async def drive(self, linear: float, angular: float) -> None:
        await self.robot_brain.send(f'wheels.speed({linear}, {angular})')

    async def stop(self) -> None:
        await self.robot_brain.send('wheels.off()')

    async def handle_core_output(self, time: float, words: list[str]) -> None:
        velocity = Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time)
        self.VELOCITY_MEASURED.emit([velocity])


class WheelsSimulation(Wheels, ModuleSimulation):
    '''This module simulates two wheels.

    Drive and stop commands impact internal velocities (linear and angular).
    A simulated pose is regularly updated with these velocities, while the velocities are emitted as an event.
    '''

    def __init__(self) -> None:
        super().__init__()

        self.pose: Pose = Pose()
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0

    async def drive(self, linear: float, angular: float) -> None:
        self.linear_velocity = linear
        self.angular_velocity = angular

    async def stop(self) -> None:
        self.linear_velocity = 0
        self.angular_velocity = 0

    async def step(self, dt: float) -> None:
        self.pose += PoseStep(linear=dt*self.linear_velocity, angular=dt*self.angular_velocity, time=rosys.time())
        velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=rosys.time())
        self.VELOCITY_MEASURED.emit([velocity])
