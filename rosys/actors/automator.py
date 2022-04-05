import asyncio
from typing import Coroutine, Optional

from .. import event, task_logger
from ..automations import Automation
from ..helpers import is_test
from . import Actor


class Automator(Actor):
    def __init__(self) -> None:
        super().__init__()
        self.automation: Optional[Automation] = None
        event.register(event.Id.PAUSE_AUTOMATION, self.pause)

    @property
    def is_stopped(self) -> bool:
        return self.automation is None or self.automation.is_stopped

    @property
    def is_running(self) -> bool:
        return self.automation is not None and self.automation.is_running

    @property
    def is_paused(self) -> bool:
        return self.automation is not None and self.automation.is_paused

    def start(self, coro: Coroutine):
        self.stop(because='new automation starts')
        self.automation = Automation(coro, self._handle_exception, on_complete=self._on_complete)
        task_logger.create_task(asyncio.wait([self.automation]), name='automation')

    def pause(self, because: str):
        if self.is_running:
            self.automation.pause()
            event.emit(event.Id.AUTOMATION_PAUSED, because)
            event.emit(event.Id.NEW_NOTIFICATION, f'automation paused because {because}')

    def resume(self):
        if self.is_paused:
            self.automation.resume()

    def stop(self, because: str):
        if not self.is_stopped:
            self.automation.stop()
            event.emit(event.Id.AUTOMATION_PAUSED, because)
            event.emit(event.Id.NEW_NOTIFICATION, f'automation stopped because {because}')

    def _handle_exception(self, e: Exception):
        self.stop(because='an exception occurred in an automation')
        if is_test:
            self.log.exception('automation failed', e)

    def _on_complete(self):
        event.emit(event.Id.AUTOMATION_COMPLETED)
        event.emit(event.Id.NEW_NOTIFICATION, f'automation completed')

    async def tear_down(self):
        await super().tear_down()
        self.stop(because='automator is shutting down')
