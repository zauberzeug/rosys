import numpy as np

from ..hardware import Wheels
from ..helpers import sleep
from ..world import World


async def drive_arc(world: World, wheels: Wheels):
    while world.robot.prediction.x < 2:
        await wheels.drive(1, np.deg2rad(25))
        await sleep(0.1)

    await wheels.stop()
