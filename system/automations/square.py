from numpy import rad2deg as deg
from numpy import deg2rad as rad
from actors.esp import Esp
from world.world import World


async def square(esp: Esp, world: World):

    while world.robot.pose.x < 2:
        await esp.drive(1, rad(0))
    while deg(world.robot.pose.yaw) < 90:
        await esp.drive(0, rad(90))
    while world.robot.pose.y < 2:
        await esp.drive(1, rad(0))
    while deg(world.robot.pose.yaw) < 180:
        await esp.drive(0, rad(90))
    while world.robot.pose.x > 0:
        await esp.drive(1, rad(0))
    while deg(world.robot.pose.yaw) < 270:
        await esp.drive(0, rad(90))
    while world.robot.pose.y > 0:
        await esp.drive(1, rad(0))

    await esp.drive(0, rad(0))
