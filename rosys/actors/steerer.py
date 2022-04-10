from enum import Enum

from .. import event
from ..hardware import Hardware
from . import Actor


class State(Enum):
    IDLE = 1
    STEERING = 2
    STOPPING = 3


class Steerer(Actor):
    interval: float = 0.05
    speed_scaling: float = 1

    def __init__(self, hardware: Hardware):
        super().__init__()
        self.hardware = hardware
        self.state = State.IDLE
        self.linear_speed = 0
        self.angular_speed = 0

    def start(self):
        self.log.info('start steering')
        self.state = State.STEERING
        event.emit(event.Id.PAUSE_AUTOMATION, 'using steerer')

    def update(self, x: float, y: float):
        if self.state == State.STEERING:
            self.linear_speed = y * self.speed_scaling
            self.angular_speed = -x * self.speed_scaling

    def stop(self):
        self.log.info('stop steering')
        self.orientation = None
        self.state = State.STOPPING

    async def step(self):
        if self.state == State.STEERING:
            await self.hardware.drive(self.linear_speed, self.angular_speed)
        elif self.state == State.STOPPING:
            await self.hardware.drive(0, 0)
            self.state = State.IDLE

    def __str__(self) -> str:
        return f'{type(self).__name__} ({self.state})'
