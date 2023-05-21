import abc

from .. import rosys
from ..event import Event
from ..geometry import Pose, PoseStep, Velocity
from ..helpers import remove_indentation
from .can import CanHardware
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Wheels(Module, abc.ABC):
    """This module represents wheels for a two-wheel differential drive.

    Wheels can be moved using the `drive` methods and provide measured velocities as an event.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.VELOCITY_MEASURED = Event()
        """new velocity measurements are available for processing (argument: list of velocities)"""

        self.linear_target_speed: float = 0.0
        self.angular_target_speed: float = 0.0

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        self.linear_target_speed = linear
        self.angular_target_speed = angular

    async def stop(self) -> None:
        await self.drive(0.0, 0.0)


class WheelsHardware(Wheels, ModuleHardware):
    """This module implements wheels hardware.

    Drive and stop commands are forwarded to a given Robot Brain.
    Velocities are read and emitted regularly.
    """

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
        lizard_code = remove_indentation(f'''
            l = ODriveMotor({can.name}, {left_can_address})
            r = ODriveMotor({can.name}, {right_can_address})
            l.m_per_tick = {m_per_tick}
            r.m_per_tick = {m_per_tick}
            l.reversed = {'true' if is_left_reversed else 'false'}
            r.reversed = {'true' if is_right_reversed else 'false'}
            {name} = ODriveWheels(l, r)
            {name}.width = {width}
        ''')
        core_message_fields = [f'{self.name}.linear_speed:3', f'{self.name}.angular_speed:3']
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        await self.robot_brain.send(f'{self.name}.speed({linear}, {angular})')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        velocity = Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time)
        self.VELOCITY_MEASURED.emit([velocity])


class WheelsSimulation(Wheels, ModuleSimulation):
    """This module simulates two wheels.

    Drive and stop commands impact internal velocities (linear and angular).
    A simulated pose is regularly updated with these velocities, while the velocities are emitted as an event.
    """

    def __init__(self) -> None:
        super().__init__()

        self.pose: Pose = Pose()
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0
        self.inertia_factor: float = 0.0
        self.is_blocking: bool = False
        self.is_slipping: bool = False

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        f = self.inertia_factor
        self.linear_velocity = 0 if self.is_blocking else f * self.linear_velocity + (1 - f) * linear
        self.angular_velocity = 0 if self.is_blocking else f * self.angular_velocity + (1 - f) * angular

    async def step(self, dt: float) -> None:
        if not self.is_slipping:
            self.pose += PoseStep(linear=dt*self.linear_velocity, angular=dt*self.angular_velocity, time=rosys.time())
        velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=rosys.time())
        self.VELOCITY_MEASURED.emit([velocity])
