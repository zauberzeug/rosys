from ..world.world import World
from ..actors.esp import Esp
from .spline import spline


async def draw(world: World, esp: Esp):

    for spline_ in world.path:
        await spline(spline_, world, esp)
