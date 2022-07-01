import logging
from enum import Enum

from ..hardware import Wheels
from ..runtime import runtime
from .automator import Automator


class State(Enum):
    IDLE = 1
    STEERING = 2
    STOPPING = 3


class Steerer:
    speed_scaling: float = 1

    def __init__(self, wheels: Wheels) -> None:
        self.log = logging.getLogger('rosys.steerer')

        self.wheels = wheels
        self.state = State.IDLE
        self.linear_speed = 0
        self.angular_speed = 0

        runtime.on_repeat(self.step, 0.05)

    def start(self) -> None:
        self.log.info('start steering')
        self.state = State.STEERING
        Automator.PAUSE_AUTOMATION.emit('using steerer')

    def update(self, x: float, y: float) -> None:
        if self.state == State.STEERING:
            self.linear_speed = y * self.speed_scaling
            self.angular_speed = -x * self.speed_scaling

    def stop(self) -> None:
        self.log.info('stop steering')
        self.orientation = None
        self.state = State.STOPPING

    async def step(self) -> None:
        if self.state == State.STEERING:
            await self.wheels.drive(self.linear_speed, self.angular_speed)
        elif self.state == State.STOPPING:
            await self.wheels.drive(0, 0)
            self.state = State.IDLE

    def __str__(self) -> str:
        return f'{type(self).__name__} ({self.state})'
