from typing import Optional
from .actor import Actor
from ..actors.esp import Esp
from ..world.world import World, NozzleState
from ..world.point import Point


class Nozzle(Actor):

    interval: float = 0.01
    last_dot: Optional[Point] = None
    last_state: Optional[NozzleState] = None

    async def step(self, world: World, esp: Esp):

        if world.nozzle != NozzleState.PULSING:
            if world.nozzle != self.last_state:
                esp.send(f'nozzle {"on" if world.nozzle == NozzleState.ON else "off"}')
                self.last_state = world.nozzle
            return

        position = world.robot.prediction.transform(world.robot.shape.point_of_interest)
        if self.last_dot is None or position.distance(self.last_dot) > 0.05:
            self.last_dot = position
            esp.send('nozzle pulse')
