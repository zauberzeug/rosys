import asyncio
from typing import Coroutine, Optional
from .. import event
from ..automations import Automation
from ..world import AutomationState
from . import Actor


class Automator(Actor):
    interval: float = 0.1

    def __init__(self, default_automation: Coroutine = None) -> None:
        super().__init__()
        self.automations: list[Automation] = []
        self.default_automation = default_automation
        event.register(event.Id.PAUSE_AUTOMATIONS, self._handle_pause_event)

    async def step(self):
        if self.automations:
            await asyncio.gather(*self.automations)
            self.stop('the last one has completed')

    def start(self, coro: Optional[Coroutine] = None):
        self.automations = [Automation(coro or self.default_automation)]
        self.world.automation_state = AutomationState.RUNNING

    def pause(self, because: Optional[str] = None):
        [a.pause() for a in self.automations]
        self.world.automation_state = AutomationState.PAUSED
        event.emit(event.Id.PAUSE_AUTOMATIONS, because)

    def resume(self):
        [a.resume() for a in self.automations]
        self.world.automation_state = AutomationState.RUNNING

    def stop(self, because: Optional[str] = None):
        # TODO: how to stop/cancel coroutines? `step()` is still gathering.
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
