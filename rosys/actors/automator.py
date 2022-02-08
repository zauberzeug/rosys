import asyncio
from typing import Coroutine, Optional
from .. import event
from ..automations import Automation
from . import Actor


class Automator(Actor):
    def __init__(self, default_automation: Coroutine = None) -> None:
        super().__init__()
        self.automation: Optional[Automation] = None
        self.default_automation = default_automation
        event.register(event.Id.PAUSE_AUTOMATIONS, self._handle_pause_event)

    @property
    def is_stopped(self) -> bool:
        return self.automation is None or self.automation.is_stopped

    @property
    def is_running(self) -> bool:
        return self.automation is not None and self.automation.is_running

    @property
    def is_paused(self) -> bool:
        return self.automation is not None and self.automation.is_paused

    async def start(self, coro: Optional[Coroutine] = None):
        await self.stop()
        self.automation = Automation(coro or self.default_automation or asyncio.sleep(0))
        asyncio.gather(self.automation)

    async def pause(self, because: Optional[str] = None):
        if self.is_running:
            self.automation.pause()
            await event.call(event.Id.PAUSE_AUTOMATIONS, because)

    def resume(self):
        if self.is_paused:
            self.automation.resume()

    async def stop(self, because: Optional[str] = None):
        if not self.is_stopped:
            self.automation.stop()
            self.automation = None
            await event.call(event.Id.PAUSE_AUTOMATIONS, because)

    async def _handle_pause_event(self, because: Optional[str] = None):
        if self.is_running:
            if because:
                await event.call(event.Id.NEW_NOTIFICATION, f'pausing automations because {because}')
            await self.pause(because)

    async def tear_down(self):
        await super().tear_down()
        self.stop()
