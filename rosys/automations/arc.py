import numpy as np

from ..hardware import Hardware
from ..helpers import sleep
from ..world import World


async def drive_arc(world: World, hardware: Hardware):
    while world.robot.prediction.x < 2:
        await hardware.drive(1, np.deg2rad(25))
        await sleep(0.1)

    await hardware.stop()
