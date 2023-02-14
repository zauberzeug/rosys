import abc

from .. import rosys
from ..event import Event
from ..geometry import Pose, PoseStep, Velocity
from .can import CanHardware
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Wheels(Module, abc.ABC):
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
    CORE_MESSAGE_FIELDS: list[str] = ['linear_speed:3', 'angular_speed:3']

    def __init__(self, robot_brain: RobotBrain, *,
                 can: CanHardware,
                 name: str = 'wheels',
                 left_can_address: int = 0x000,
                 right_can_address: int = 0x100,
                 m_per_tick: float = 0.01,
                 width: float = 0.5,
                 is_left_reversed: bool = False,
                 is_right_reversed: bool = False) -> None:
        self.name = name
        lizard_code = f'''
            l = ODriveMotor({can.name}, {left_can_address})
            r = ODriveMotor({can.name}, {right_can_address})
            l.m_per_tick = {m_per_tick}
            r.m_per_tick = {m_per_tick}
            l.reversed = {'true' if is_left_reversed else 'false'}
            r.reversed = {'true' if is_right_reversed else 'false'}
            {name} = ODriveWheels(l, r)
            {name}.width = {width}
        '''
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def drive(self, linear: float, angular: float) -> None:
        await self.robot_brain.send(f'{self.name}.speed({linear}, {angular})')

    async def stop(self) -> None:
        await self.robot_brain.send(f'{self.name}.off()')

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
