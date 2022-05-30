import abc
import logging
from typing import Optional

from ..world import World
from .app_controls import AppControls


class Hardware(abc.ABC):

    def __init__(self, world: World):
        self.world = world
        self.name = __name__[:-8] + self.__class__.__name__
        self.log = logging.getLogger(self.name)
        self.app_controls: Optional[AppControls] = None

    @abc.abstractmethod
    async def configure(self):
        '''Send current configuration to the hardware.'''
        return

    @abc.abstractmethod
    async def restart(self):
        '''Restarts hardware'''
        return

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float):
        '''Send drive command to the hardware'''
        return

    @abc.abstractmethod
    async def stop(self):
        '''Stop the driving wheels'''
        return

    @abc.abstractmethod
    async def update(self):
        '''Called by actors to update the world'''
        return

    async def startup(self):
        return

    async def tear_down(self):
        return

    @property
    def is_simulation(self) -> bool:
        return True

    @property
    def is_real(self) -> bool:
        return not self.is_simulation
