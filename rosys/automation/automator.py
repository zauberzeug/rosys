import asyncio
import logging
from collections.abc import Callable, Coroutine
from typing import Literal

from nicegui import Event

from .. import rosys
from ..driving import Steerer
from .automation import Automation


class Automator:
    """An automator allows running automations, i.e. coroutines that can be paused and resumed.

    See [Click-and-drive](https://rosys.io/examples/click-and-drive/) for a simple example of an automation.

    _steerer_: If provided, manually steering the robot will pause a currently running automation.

    _default_automation_: If provided, it allows the automator to start a new automation without passing an automation
    (e.g. via an "Play"-button like offered by the [automation controls](https://rosys.io/reference/rosys/automation/#rosys.automation.automation_controls)).
    The passed function should return a new coroutine on every call (see [Play-pause-stop](https://rosys.io/examples/play-pause-stop/) example).

    _on_interrupt_: Optional callback that will be called when an automation pauses or stops (the cause is provided as string parameter).

    _notify_: If True, the automator will send notifications when an automation starts, pauses, resumes, stops or fails.
    """

    def __init__(self,
                 steerer: Steerer | None, *,
                 default_automation: Callable | None = None,
                 on_interrupt: Callable | None = None,
                 notify: bool = True) -> None:
        self.AUTOMATION_STARTED = Event[[]]()
        """an automation has been started"""

        self.AUTOMATION_PAUSED = Event[str]()
        """an automation has been paused (string argument: description of the cause)"""

        self.AUTOMATION_RESUMED = Event[[]]()
        """an automation has been resumed"""

        self.AUTOMATION_STOPPED = Event[str]()
        """an automation has been stopped (string argument: description of the cause)"""

        self.AUTOMATION_FAILED = Event[str]()
        """an automation has failed to complete (string argument: description of the cause)"""

        self.AUTOMATION_COMPLETED = Event[[]]()
        """an automation has been completed"""

        self.log = logging.getLogger('rosys.automator')

        self.default_automation = default_automation
        self._on_interrupt = on_interrupt
        self.notify = notify

        self.enabled: bool = True
        self.automation: Automation | None = None

        if steerer:
            steerer.STEERING_STARTED.subscribe(lambda: self.pause(because='steering started'))

        self.AUTOMATION_PAUSED.subscribe(lambda _: self._handle_interrupt())
        self.AUTOMATION_STOPPED.subscribe(lambda _: self._handle_interrupt(stop=True))
        self.AUTOMATION_FAILED.subscribe(lambda _: self._handle_interrupt(stop=True))

        rosys.on_shutdown(lambda: self.stop(because='automator is shutting down'))

    @property
    def is_stopped(self) -> bool:
        return self.automation is None or self.automation.is_stopped

    @property
    def is_stopping(self) -> bool:
        return self.automation is not None and self.automation.is_stopping

    @property
    def is_running(self) -> bool:
        return self.automation is not None and self.automation.is_running

    @property
    def is_paused(self) -> bool:
        return self.automation is not None and self.automation.is_paused

    @property
    def is_pausing(self) -> bool:
        return self.automation is not None and self.automation.is_pausing

    async def _handle_interrupt(self, *, stop: bool = False) -> None:
        assert self.automation is not None
        while self.automation.is_running:
            await rosys.sleep(0.1)
        if self._on_interrupt:
            if asyncio.iscoroutinefunction(self._on_interrupt):
                await self._on_interrupt()
            else:
                self._on_interrupt()
        if stop:
            self.automation = None

    def start(self, coro: Coroutine | None = None, *, paused: bool = False) -> None:
        """Starts a new automation.

        :param coro: the coroutine to start, if None the default automation will be used
        :param paused: whether to start the automation paused
        """
        if coro is None:
            assert self.default_automation is not None
            self.start(self.default_automation(), paused=paused)
            return
        if not self.enabled:
            coro.close()
            return
        self.stop(because='new automation starts')
        self.automation = Automation(coro, self._handle_exception, on_complete=self._on_complete)
        rosys.background_tasks.create(self.automation.run(), name='automation')  # type: ignore
        self.AUTOMATION_STARTED.emit()
        self._notify('automation started')
        if paused:
            self.automation.pause()

    def pause(self, because: str) -> None:
        """Pauses the current automation.

        :param because: the reason for pausing the automation
        """
        if self.is_pausing or self.is_stopping:
            return
        if self.is_running:
            assert self.automation is not None
            self.automation.pause()
            self.AUTOMATION_PAUSED.emit(because)
            self._notify(f'automation paused because {because}')

    def resume(self) -> None:
        """Resumes the current automation."""
        if not self.enabled:
            return
        if self.is_paused:
            assert self.automation is not None
            self.automation.resume()
            self.AUTOMATION_RESUMED.emit()
            self._notify('automation resumed')

    def stop(self, because: str) -> None:
        """Stops the current automation.

        :param because: the reason for stopping the automation
        """
        if self.is_pausing or self.is_stopping:
            return
        if self.is_running or self.is_paused:
            assert self.automation is not None
            self.automation.stop()
            self.AUTOMATION_STOPPED.emit(because)
            self._notify(f'automation stopped because {because}')

    def abort(self, because: str) -> None:
        """Stops the current automation because of a failure.

        :param because: the reason for aborting the automation
        """
        if self.is_stopped:
            return
        assert self.automation is not None
        self.automation.stop()
        self.AUTOMATION_FAILED.emit(because)
        self._notify(f'automation aborted because {because}')

    def enable(self) -> None:
        """Enables the automator.

        It is enabled by default.
        It can be disabled by calling `disable()`.
        """
        self.enabled = True

    def disable(self, because: str) -> None:
        """Disables the automator.

        No automations can be started while the automator is disabled.
        If an automation is running or paused it will be stopped.

        :param because: the reason for disabling the automator
        """
        self.stop(because)
        self.enabled = False

    def set_default_automation(self, default_automation: Callable | None) -> None:
        """Sets the default automation.

        :param default_automation: the default automation to use
        """
        self.default_automation = default_automation

    def _handle_exception(self, e: Exception) -> None:
        self.abort(because=f'an exception occurred in an automation{f": {e}" if str(e) else ""}')
        if rosys.is_test:
            self.log.exception('automation failed')

    def _on_complete(self) -> None:
        self.automation = None
        self.AUTOMATION_COMPLETED.emit()
        self._notify('automation completed', 'positive')

    def _notify(self, message: str,
                type: Literal[  # pylint: disable=redefined-builtin
                    'positive',
                    'negative',
                    'warning',
                    'info',
                    'ongoing',
                ] | None = None) -> None:
        if self.notify:
            rosys.notify(message, type)
        else:
            rosys.log.info(message)
