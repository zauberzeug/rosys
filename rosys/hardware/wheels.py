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

        self.VELOCITY_MEASURED = Event[list[Velocity]]()

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

    def __init__(self, width: float = 0.5) -> None:
        super().__init__()

        self.width: float = width
        """The distance between the wheels -- used to calculate actual drift when slip_factor_* is used."""

        self.pose: Pose = Pose(time=rosys.time())
        """Provides the actual pose of the robot which can alter due to slippage."""

        self.linear_velocity: float = 0
        """The current linear velocity of the robot."""

        self.angular_velocity: float = 0
        """The current angular velocity of the robot."""

        self.inertia_factor: float = 0.0
        """The factor of inertia for the wheels (0 = no inertia, 1 = full inertia)."""

        self.friction_factor: float = 0.0
        """The factor of friction for the wheels (0 = no friction, 1 = full friction)."""

        self.is_blocking: bool = False
        """If True, the wheels are blocking and the robot will not move."""

        self.slip_factor_left: float = 0
        """The factor of slippage for the left wheel (0 = no slippage, 1 = full slippage)."""

        self.slip_factor_right: float = 0
        """The factor of slippage for the right wheel (0 = no slippage, 1 = full slippage)."""

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        f = self.inertia_factor
        self.linear_velocity = 0 if self.is_blocking else f * self.linear_velocity + (1 - f) * linear
        self.angular_velocity = 0 if self.is_blocking else f * self.angular_velocity + (1 - f) * angular

    async def step(self, dt: float) -> None:
        self.linear_velocity *= 1 - self.friction_factor
        self.angular_velocity *= 1 - self.friction_factor
        left_speed = self.linear_velocity - self.angular_velocity * self.width / 2
        right_speed = self.linear_velocity + self.angular_velocity * self.width / 2
        left_speed *= 1 - self.slip_factor_left
        right_speed *= 1 - self.slip_factor_right
        self.pose += PoseStep(linear=dt * (left_speed + right_speed) / 2,
                              angular=dt * (right_speed - left_speed) / self.width,
                              time=rosys.time())
        velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=self.pose.time)
        self.VELOCITY_MEASURED.emit([velocity])
