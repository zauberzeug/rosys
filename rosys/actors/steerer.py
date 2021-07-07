from enum import Enum
from rosys.actors.esp import Esp
from rosys.actors.actor import Actor


class State(Enum):
    IDLE = 1
    INITIALIZING = 2
    STEERING = 3
    STOPPING = 4


class Steerer(Actor):

    interval = 0.05

    def __init__(self):

        self.state = State.IDLE
        self.orientation = None
        self.linear_speed = 0
        self.angular_speed = 0

    def start(self):

        self.state = State.INITIALIZING

    def update(self, x: float, y: float):

        if self.state == State.INITIALIZING:
            squared_distance = x**2 + y**2
            dead_zone = 0.1
            if squared_distance > dead_zone**2:
                is_down = y < 0 and abs(y) > abs(x)
                self.orientation = -1 if is_down else 1
                self.state = State.STEERING

        if self.state == State.STEERING:
            self.linear_speed = y
            self.angular_speed = -x * self.orientation

    def stop(self):

        self.orientation = None
        self.state = State.STOPPING

    async def step(self, esp: Esp):

        if self.state == State.STEERING:
            await esp.drive(self.linear_speed, self.angular_speed)

        if self.state == State.STOPPING:
            await esp.drive(0, 0)
            self.state = State.IDLE

    def __str__(self) -> str:
        return f'{type(self).__name__} ({self.state})'
