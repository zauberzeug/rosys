import numpy as np
from ..actors.esp import Esp
from ..world.world import World


async def drive_arc(world: World, esp: Esp):
    while world.robot.prediction.x < 2:
        await esp.drive(1, np.deg2rad(25))

    await esp.drive(0, np.deg2rad(0))
