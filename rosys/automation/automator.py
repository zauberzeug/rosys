import asyncio
import logging
from typing import Callable, Coroutine, Optional

import rosys

from ..driving import Drivable, Steerer
from ..event import Event
from ..task_logger import create_task
from .automation import Automation


class Automator:
    '''An automator allows running automations, i.e. coroutines that can be paused and resumed.

    A default automation can be specified, so that the automator can later be started (e.g. via an automation controls UI element) without passing an automation.
    Optional wheels (or any stoppable hardware representation) will be stopped when an automation pauses or stops.
    Manually steering the robot using an optional steerer pauses a currently running automation.
    '''

    def __init__(self,
                 wheels: Optional[Drivable],
                 steerer: Optional[Steerer], *,
                 default_automation: Optional[Callable] = None) -> None:
        self.AUTOMATION_STARTED = Event()
        '''an automation has been started'''

        self.AUTOMATION_PAUSED = Event()
        '''an automation has been paused (string argument: description of the cause)'''

        self.AUTOMATION_RESUMED = Event()
        '''an automation has been resumed'''

        self.AUTOMATION_STOPPED = Event()
        '''an automation has been stopped (string argument: description of the cause)'''

        self.AUTOMATION_FAILED = Event()
        '''an automation has failed to complete (string argument: description of the cause)'''

        self.AUTOMATION_COMPLETED = Event()
        '''an automation has been completed'''

        self.log = logging.getLogger('rosys.automator')

        self.default_automation = default_automation

        self.enabled: bool = True
        self.automation: Optional[Automation] = None

        if steerer:
            steerer.STEERING_STARTED.register(lambda: self.pause(because='steering started'))

        if wheels:
            self.AUTOMATION_PAUSED.register(lambda _: create_task(wheels.stop()))
            self.AUTOMATION_STOPPED.register(lambda _: create_task(wheels.stop()))

        rosys.on_shutdown(lambda: self.stop(because='automator is shutting down'))

    @property
    def is_stopped(self) -> bool:
        return self.automation is None or self.automation.is_stopped

    @property
    def is_running(self) -> bool:
        return self.automation is not None and self.automation.is_running

    @property
    def is_paused(self) -> bool:
        return self.automation is not None and self.automation.is_paused

    def start(self, coro: Optional[Coroutine] = None) -> None:
        if coro is None:
            self.start(self.default_automation())
            return
        if not self.enabled:
            coro.close()
            return
        self.stop(because='new automation starts')
        self.automation = Automation(coro, self._handle_exception, on_complete=self._on_complete)
        create_task(asyncio.wait([self.automation]), name='automation')
        self.AUTOMATION_STARTED.emit()
        rosys.notify('automation started')

    def pause(self, because: str) -> None:
        if self.is_running:
            self.automation.pause()
            self.AUTOMATION_PAUSED.emit(because)
            rosys.notify(f'automation paused because {because}')

    def resume(self) -> None:
        if not self.enabled:
            return
        if self.is_paused:
            self.automation.resume()
            self.AUTOMATION_RESUMED.emit()
            rosys.notify('automation resumed')

    def stop(self, because: str) -> None:
        if not self.is_stopped:
            self.automation.stop()
            self.AUTOMATION_STOPPED.emit(because)
            rosys.notify(f'automation stopped because {because}')

    def enable(self) -> None:
        self.enabled = True

    def disable(self, because: str) -> None:
        self.stop(because)
        self.enabled = False

    def _handle_exception(self, e: Exception) -> None:
        self.stop(because='an exception occurred in an automation')
        self.AUTOMATION_FAILED.emit(str(e))
        rosys.notify('automation failed')
        if rosys.is_test:
            self.log.exception('automation failed', e)

    def _on_complete(self) -> None:
        self.AUTOMATION_COMPLETED.emit()
        rosys.notify('automation completed')
