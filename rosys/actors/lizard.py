from .. import event
from ..hardware.hardware import Hardware
from . import Actor


class Lizard(Actor):
    interval: float = 0.01

    def __init__(self, hardware: Hardware):
        super().__init__()
        self.hardware = hardware
        self.last_step = None
        event.register(event.Id.PAUSE_AUTOMATIONS, self._handle_pause)

    async def step(self):
        await self.ensure_responsiveness()
        await self.hardware.update()
        await event.call(event.Id.NEW_MACHINE_DATA)

    async def _handle_pause(self, reason: str):
        await self.hardware.stop()

    async def ensure_responsiveness(self):
        dt = self.world.time - self.last_step if self.last_step is not None else 0
        if dt > 1:
            msg = 'esp serial communication can not be guaranteed (>= 1 sec)'
            self.log.error(msg + '; aborting automations')
            await self.pause_automations(because=msg)
        elif dt > 0.1:
            self.log.warn('esp serial communication is slow (>= 100 ms)')
        self.last_step = self.world.time
