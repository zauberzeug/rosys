import sys
import asyncio
from world.mode import Mode
from world.world import World
import time


class Actor:

    def __init__(self, world: World):

        self.world = world
