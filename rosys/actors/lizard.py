from collections import deque
from .. import event
from ..hardware import Hardware
from . import Actor


class Lizard(Actor):
    interval: float = 0.01

    def __init__(self, hardware: Hardware):
        super().__init__()
        self.hardware = hardware
        self.last_step = None
        event.register(event.Id.AUTOMATION_PAUSED, self._handle_pause)
        self.responsiveness_stats: deque = deque(maxlen=100)
        self.update_stats: deque = deque(maxlen=100)
        self.processing_stats: deque = deque(maxlen=100)

    async def step(self):
        await self.ensure_responsiveness()
        t = self.world.time
        await self.hardware.update()
        self.update_stats.append(self.world.time - t)
        t = self.world.time
        await event.call(event.Id.NEW_MACHINE_DATA)
        dt = self.world.time - t
        if dt > 0.02:
            self.log.warning(f'processing machine data took {dt:.2f} s')
        self.processing_stats.append(dt)

    async def _handle_pause(self, reason: str):
        await self.hardware.stop()

    async def ensure_responsiveness(self):
        dt = self.world.time - self.last_step if self.last_step is not None else 0
        if dt > 1:
            msg = f'esp serial communication can not be guaranteed ({dt:.2f} s since last step)'
            self.log.error(msg + '; aborting automation')
            await self.pause_automation(because=msg)
        elif dt > 0.1:
            self.log.warning(f'esp serial communication is slow ({dt:.2f} s since last step)')
        self.last_step = self.world.time
        self.responsiveness_stats.append(dt)
