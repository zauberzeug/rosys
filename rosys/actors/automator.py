import asyncio
import logging
from typing import Coroutine, Optional

from .. import task_logger
from ..automation import Automation
from ..event import Event
from ..runtime import runtime


class Automator:
    AUTOMATION_STARTED = Event()
    '''an automation has been started'''

    PAUSE_AUTOMATION = Event()
    '''pause a running automation (string argument: description of the cause)'''

    STOP_AUTOMATION = Event()
    '''stop a running automation (string argument: description of the cause)'''

    AUTOMATION_PAUSED = Event()
    '''an automation has been paused (string argument: description of the cause)'''

    AUTOMATION_RESUMED = Event()
    '''an automation has been resumed'''

    AUTOMATION_STOPPED = Event()
    '''an automation has been stopped (string argument: description of the cause)'''

    AUTOMATION_FAILED = Event()
    '''an automation has failed to complete (string argument: description of the cause)'''

    AUTOMATION_COMPLETED = Event()
    '''an automation has been completed'''

    def __init__(self) -> None:
        self.log = logging.getLogger(self.__class__.__name__)

        self.enabled: bool = True
        self.automation: Optional[Automation] = None

        self.PAUSE_AUTOMATION.register(self.pause)
        self.STOP_AUTOMATION.register(self.stop)

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
        self.AUTOMATION_STARTED.emit()
        runtime.notify('automation started')

    def pause(self, because: str) -> None:
        if self.is_running:
            self.automation.pause()
            self.AUTOMATION_PAUSED.emit(because)
            runtime.notify(f'automation paused because {because}')

    def resume(self) -> None:
        if not self.enabled:
            return
        if self.is_paused:
            self.automation.resume()
            self.AUTOMATION_RESUMED.emit()
            runtime.notify('automation resumed')

    def stop(self, because: str) -> None:
        if not self.is_stopped:
            self.automation.stop()
            self.AUTOMATION_STOPPED.emit(because)
            runtime.notify(f'automation stopped because {because}')

    def enable(self) -> None:
        self.enabled = True

    def disable(self, because: str) -> None:
        self.stop(because)
        self.enabled = False

    def _handle_exception(self, e: Exception) -> None:
        self.stop(because='an exception occurred in an automation')
        self.AUTOMATION_FAILED.emit(str(e))
        runtime.notify('automation failed')
        if runtime.is_test:
            self.log.exception('automation failed', e)

    def _on_complete(self) -> None:
        self.AUTOMATION_COMPLETED.emit()
        runtime.notify('automation completed')
