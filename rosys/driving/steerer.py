import logging
from enum import Enum

from rosys.event import Event

from .. import rosys
from .drivable import Drivable


class State(Enum):
    IDLE = 1
    STEERING = 2
    STOPPING = 3


class Steerer:
    """The steerer module translates x-y information (e.g. from a joystick) to linear/angular velocities sent to the robot.

    The wheels module can be any drivable hardware representation.
    Changing the steering state emits events that can be used to react to manual user interaction.
    """

    def __init__(self, wheels: Drivable, speed_scaling: float = 1.0) -> None:
        self.STEERING_STARTED = Event[[]]()
        """steering has started"""

        self.STEERING_STOPPED = Event[[]]()
        """steering has stopped"""

        self.log = logging.getLogger('rosys.steerer')

        self.wheels = wheels
        self.speed_scaling = speed_scaling
        self.state = State.IDLE
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        rosys.on_repeat(self.step, 0.05)

    def start(self) -> None:
        self.log.info('start steering')
        self.state = State.STEERING
        self.STEERING_STARTED.emit()

    def update(self, x: float, y: float) -> None:
        if self.state == State.STEERING:
            self.linear_speed = y * self.speed_scaling
            self.angular_speed = -x * self.speed_scaling

    def stop(self) -> None:
        if self.state != State.STEERING:
            return
        self.log.info('stop steering')
        self.state = State.STOPPING
        self.STEERING_STOPPED.emit()

    async def step(self) -> None:
        if self.state == State.STEERING:
            await self.wheels.drive(self.linear_speed, self.angular_speed)
        elif self.state == State.STOPPING:
            await self.wheels.stop()
            self.state = State.IDLE

    def set_speed_scaling(self, speed_scaling: float) -> None:
        self.speed_scaling = speed_scaling

    def __str__(self) -> str:
        return f'{type(self).__name__} ({self.state})'
