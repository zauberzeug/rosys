from typing import Optional
from .actor import Actor
from ..world.world import World, NozzleState
from ..world.point import Point


class Nozzle(Actor):

    interval: float = 0.01
    last_dot: Optional[Point] = None
    dot_distance: float = 0.03

    async def step(self, world: World):

        if world.nozzle != NozzleState.PULSING:
            return

        position = world.robot.prediction.transform(world.robot.shape.point_of_interest)
        if self.last_dot is None or position.distance(self.last_dot) > 0.1:
            ic('dot')
            self.last_dot = position
