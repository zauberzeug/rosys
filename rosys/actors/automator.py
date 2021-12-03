from enum import auto
from typing import Coroutine, Optional

from rosys.actors.esp import Esp
from ..world.world import World, AutomationState
from ..automations import drive_path
from .actor import Actor
from .. import event


class Automator(Actor):
    interval: float = 0.1

    def __init__(self, esp: Esp) -> None:
        super().__init__()
        self.routines = []
        self.esp = esp
        event.register(event.Id.PAUSE_AUTOMATIONS, self._pause)

    def add(self, coro: Coroutine):
        self.routines.append(coro)

    def replace(self, coro: Coroutine):
        self.routines.clear()
        self.add(coro)

    async def step(self):
        if not self.routines and not self.world.path:
            self.world.automation_state = AutomationState.DISABLED

        if self.world.automation_state == AutomationState.DISABLED:
            if not self.routines and self.world.path:
                self.add(drive_path(self.world, self.esp))
            if self.routines:
                self.world.automation_state = AutomationState.STOPPED

        if self.world.automation_state != AutomationState.RUNNING:
            return

        for coro in self.routines:
            try:
                coro.send(None)
            except StopIteration:
                self.routines.remove(coro)
                if not self.routines:
                    await self.pause_automations(because='the last one has completed')
            except:
                await self.pause_automations(because='an exception occurred in an automation')
                self.routines.clear()
                self.log.exception(f'paused and cleared automations due to exception in {coro}')

    async def _pause(self, because: Optional[str] = None):
        '''Pauses the automation. 

        Only to be used internally. The proper way is to use runtime.pause(...) or fire event.Id.PAUSE_AUTOMATION. See rosys.io/automations.
        '''
        if self.world.automation_state == AutomationState.PAUSED:
            return
        self.world.automation_state = AutomationState.PAUSED
        if because:
            await event.call(event.Id.NEW_NOTIFICATION, f'pausing automations because {because}')
