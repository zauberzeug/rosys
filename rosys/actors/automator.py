import asyncio
from typing import Coroutine, Optional
from .. import event
from ..automations import Automation
from ..world import AutomationState
from . import Actor


class Automator(Actor):
    def __init__(self, default_automation: Coroutine = None) -> None:
        super().__init__()
        self.automations: list[Automation] = []
        self.default_automation = default_automation
        event.register(event.Id.PAUSE_AUTOMATIONS, self._handle_pause_event)

    def start(self, coro: Optional[Coroutine] = None):
        self.automations = [Automation(coro or self.default_automation)]
        [a.stop() for a in self.automations]
        asyncio.gather(*self.automations)
        self.world.automation_state = AutomationState.RUNNING

    def pause(self, because: Optional[str] = None):
        [a.pause() for a in self.automations]
        self.world.automation_state = AutomationState.PAUSED
        event.emit(event.Id.PAUSE_AUTOMATIONS, because)

    def resume(self):
        [a.resume() for a in self.automations]
        self.world.automation_state = AutomationState.RUNNING

    def stop(self, because: Optional[str] = None):
        [a.stop() for a in self.automations]
        self.automations.clear()
        self.world.automation_state = AutomationState.STOPPED
        event.emit(event.Id.PAUSE_AUTOMATIONS, because)

    def _handle_pause_event(self, because: Optional[str] = None):
        if self.world.automation_state == AutomationState.RUNNING:
            if because:
                event.emit(event.Id.NEW_NOTIFICATION, f'pausing automations because {because}')
            self.pause(because)

    async def tear_down(self):
        await super().tear_down()
        self.stop()
