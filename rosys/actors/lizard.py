from collections import deque

from .. import event
from ..core import core
from ..hardware import Hardware
from ..lifecycle import on_repeat
from . import Actor


class Lizard(Actor):

    def __init__(self, hardware: Hardware):
        super().__init__()
        self.hardware = hardware
        self.last_step = None
        event.register(event.Id.AUTOMATION_PAUSED, self._stop)
        event.register(event.Id.AUTOMATION_STOPPED, self._stop)
        event.register(event.Id.AUTOMATION_FAILED, self._stop)
        self.responsiveness_stats: deque = deque(maxlen=100)
        self.update_stats: deque = deque(maxlen=100)
        self.processing_stats: deque = deque(maxlen=100)
        on_repeat(self.step, 0.01)

    async def step(self):
        if self.hardware.is_real:
            await self.ensure_responsiveness()
        t = core.time
        await self.hardware.update()
        self.update_stats.append(core.time - t)
        t = core.time
        await event.call(event.Id.NEW_MACHINE_DATA)
        dt = core.time - t
        if dt > 0.02:
            self.log.warning(f'processing machine data took {dt:.2f} s')
        self.processing_stats.append(dt)

    async def _stop(self, _):
        await self.hardware.stop()

    async def ensure_responsiveness(self):
        dt = core.time - self.last_step if self.last_step is not None else 0
        if dt > 1:
            self.log.error(f'esp serial communication can not be guaranteed ({dt:.2f} s since last step)')
        elif dt > 0.1:
            self.log.warning(f'esp serial communication is slow ({dt:.2f} s since last step)')
        self.last_step = core.time
        self.responsiveness_stats.append(dt)
