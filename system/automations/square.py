import numpy as np
from actors.esp import Esp
from world.world import World


async def square(world: World, esp: Esp):

    while world.robot.pose.x < 2:
        await esp.drive(1, 0)
    while world.robot.pose.yaw < np.deg2rad(90):
        await esp.drive(0, np.deg2rad(90))
    while world.robot.pose.y < 2:
        await esp.drive(1, 0)
    while world.robot.pose.yaw < np.deg2rad(180):
        await esp.drive(0, np.deg2rad(90))
    while world.robot.pose.x > 0:
        await esp.drive(1, 0)
    while world.robot.pose.yaw < np.deg2rad(270):
        await esp.drive(0, np.deg2rad(90))
    while world.robot.pose.y > 0:
        await esp.drive(1, 0)

    await esp.drive(0, 0)
