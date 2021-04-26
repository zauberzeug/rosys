from numpy import rad2deg as deg
from numpy import deg2rad as rad

from actors.esp import Esp
from world.world import World
from actors.actor import Actor
from actors.guard import Guard


class SquareDriver(Actor):

    async def once(self, esp: Esp, guard: Guard):
        await esp.drive(1, rad(0))
        await guard.condition(lambda r: r.pose.x >= 2)
        await esp.drive(0, rad(90))
        await guard.condition(lambda r: deg(r.pose.yaw) >= 90)
        await esp.drive(1, rad(0))
        await guard.condition(lambda r: r.pose.y >= 2)
        await esp.drive(0, rad(90))
        await guard.condition(lambda r: deg(r.pose.yaw) >= 180)
        await esp.drive(1, rad(0))
        await guard.condition(lambda r: r.pose.x <= 0)
        await esp.drive(0, rad(90))
        await guard.condition(lambda r: deg(r.pose.yaw) >= 270)
        await esp.drive(1, rad(0))
        await guard.condition(lambda r: r.pose.y <= 0)
        await esp.drive(0, rad(0))
