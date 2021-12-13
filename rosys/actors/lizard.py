from .. import event
from ..hardware import Hardware
from .actor import Actor


class Lizard(Actor):
    interval: float = 0.01

    def __init__(self, hardware: Hardware):
        super().__init__()
        self.hardware = hardware

        async def handle_pause(reason: str):
            self.hardware.stop()

        event.register(event.Id.PAUSE_AUTOMATIONS, handle_pause)

    async def step(self):
        await self.hardware.update()
        await event.call(event.Id.NEW_MACHINE_DATA)
