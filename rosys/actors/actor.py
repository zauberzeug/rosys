import logging
from typing import Optional
from .. import event
from ..world import World


class Actor:
    world: World
    interval: Optional[float] = None

    def __init__(self) -> None:
        self.name = __name__[:-5] + self.__class__.__name__
        self.log = logging.getLogger(self.name)

    async def startup(self):
        pass

    async def step(self):
        pass

    async def tear_down(self):
        pass

    async def pause_automation(self, *, because: str):
        await event.call(event.Id.PAUSE_AUTOMATION, because)

    async def notify(self, message: str):
        await event.call(event.Id.NEW_NOTIFICATION, message)

    def __str__(self) -> str:
        return type(self).__name__
