from .. import event
from ..hardware import Hardware
from . import Actor


class Lizard(Actor):
    interval: float = 0.01

    def __init__(self, hardware: Hardware):
        super().__init__()
        self.hardware = hardware
        event.register(event.Id.PAUSE_AUTOMATIONS, self._handle_pause)

    async def step(self):
        await self.hardware.update()
        await event.call(event.Id.NEW_MACHINE_DATA)

    async def _handle_pause(self, reason: str):
        await self.hardware.stop()
