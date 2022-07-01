import asyncio
import logging
from typing import Coroutine, Optional

from .. import event, task_logger
from ..automation import Automation
from ..runtime import runtime


class Automator:

    def __init__(self) -> None:
        self.log = logging.getLogger(self.__class__.__name__)

        self.enabled: bool = True
        self.automation: Optional[Automation] = None

        event.register(event.Id.PAUSE_AUTOMATION, self.pause)
        event.register(event.Id.STOP_AUTOMATION, self.stop)

        runtime.on_shutdown(lambda: self.stop(because='automator is shutting down'))

    @property
    def is_stopped(self) -> bool:
        return self.automation is None or self.automation.is_stopped

    @property
    def is_running(self) -> bool:
        return self.automation is not None and self.automation.is_running

    @property
    def is_paused(self) -> bool:
        return self.automation is not None and self.automation.is_paused

    def start(self, coro: Coroutine) -> None:
        if not self.enabled:
            coro.close()
            return
        self.stop(because='new automation starts')
        self.automation = Automation(coro, self._handle_exception, on_complete=self._on_complete)
        task_logger.create_task(asyncio.wait([self.automation]), name='automation')
        event.emit(event.Id.AUTOMATION_STARTED)
        event.emit(event.Id.NEW_NOTIFICATION, f'automation started')

    def pause(self, because: str) -> None:
        if self.is_running:
            self.automation.pause()
            event.emit(event.Id.AUTOMATION_PAUSED, because)
            event.emit(event.Id.NEW_NOTIFICATION, f'automation paused because {because}')

    def resume(self) -> None:
        if not self.enabled:
            return
        if self.is_paused:
            self.automation.resume()
            event.emit(event.Id.AUTOMATION_RESUMED)
            event.emit(event.Id.NEW_NOTIFICATION, f'automation resumed')

    def stop(self, because: str) -> None:
        if not self.is_stopped:
            self.automation.stop()
            event.emit(event.Id.AUTOMATION_STOPPED, because)
            event.emit(event.Id.NEW_NOTIFICATION, f'automation stopped because {because}')

    def enable(self) -> None:
        self.enabled = True

    def disable(self, because: str) -> None:
        self.stop(because)
        self.enabled = False

    def _handle_exception(self, e: Exception) -> None:
        self.stop(because='an exception occurred in an automation')
        event.emit(event.Id.AUTOMATION_FAILED, str(e))
        event.emit(event.Id.NEW_NOTIFICATION, f'automation failed')
        if runtime.is_test:
            self.log.exception('automation failed', e)

    def _on_complete(self) -> None:
        event.emit(event.Id.AUTOMATION_COMPLETED)
        event.emit(event.Id.NEW_NOTIFICATION, f'automation completed')
