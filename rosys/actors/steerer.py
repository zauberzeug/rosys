from enum import Enum
from rosys.actors.esp import Esp
from rosys.actors.actor import Actor


class Steerer(Actor):

    interval = 0.05

    def __init__(self):

        self.orientation = None
        self.linear_speed = 0
        self.angular_speed = 0

    def start(self):

        self.state = Actor.State.activating

    def update(self, x: float, y: float):

        if self.state == Actor.State.activating:
            squared_distance = x**2 + y**2
            dead_zone = 0.1
            if squared_distance > dead_zone**2:
                is_down = y < 0 and abs(y) > abs(x)
                self.orientation = -1 if is_down else 1
                self.state = Actor.State.active

        if self.state == Actor.State.active:
            self.linear_speed = y
            self.angular_speed = -x * self.orientation

    def stop(self):

        self.orientation = None
        self.state = Actor.State.deactivating

    async def step(self, esp: Esp):

        if self.state == Actor.State.active:
            await esp.drive(self.linear_speed, self.angular_speed)

        if self.state == Actor.State.deactivating:
            await esp.drive(0, 0)
            self.state = Actor.State.inactive
