from actors.esp import Esp
from world.world import World
from actors.actor import Actor
import numpy as np


class ArcDriver(Actor):

    async def every_100_ms(self, world: World, esp: Esp):
        if world.robot.pose.x < 2:
            await esp.drive(1, np.deg2rad(25))
        else:
            await esp.drive(0, np.deg2rad(0))
