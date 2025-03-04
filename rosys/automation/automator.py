import logging
from collections.abc import Callable, Coroutine
from typing import cast

from .. import rosys
from ..driving import Steerer
from ..event import Event
from .automation import Automation


class Automator:
    """An automator allows running automations, i.e. coroutines that can be paused and resumed.

    See [Click-and-drive](https://rosys.io/examples/click-and-drive/) for a simple example of an automation.

    _steerer_: If provided, manually steering the robot will pause a currently running automation.

    _default_automation_: If provided, it allows the automator to start a new automation without passing an automation
    (e.g. via an "Play"-button like offered by the [automation controls](https://rosys.io/reference/rosys/automation/#rosys.automation.automation_controls)).
    The passed function should return a new coroutine on every call (see [Play-pause-stop](https://rosys.io/examples/play-pause-stop/) example).

    _on_interrupt_: Optional callback that will be called when an automation pauses or stops (the cause is provided as string parameter).
    """

    def __init__(self,
                 steerer: Steerer | None, *,
                 default_automation: Callable | None = None,
                 on_interrupt: Callable | None = None) -> None:
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

        self.enabled: bool = True
        self.automation: Automation | None = None

        if steerer:
            steerer.STEERING_STARTED.register(lambda: self.pause(because='steering started'))

        if on_interrupt:
            self.AUTOMATION_PAUSED.register(lambda _: cast(Callable, on_interrupt)())
            self.AUTOMATION_STOPPED.register(lambda _: cast(Callable, on_interrupt)())

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

    def start(self, coro: Coroutine | None = None, *, paused: bool = False) -> None:
        """Starts a new automation.

        You can pass any coroutine.
        The automator will make sure it can be paused, resumed and stopped.
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
        rosys.notify('automation started')
        if paused:
            self.automation.pause()

    def pause(self, because: str) -> None:
        """Pauses the current automation.

        You need to provide a cause which will be used as notification message.
        """
        if self.is_running:
            assert self.automation is not None
            self.automation.pause()
            self.AUTOMATION_PAUSED.emit(because)
            rosys.notify(f'automation paused because {because}')

    def resume(self) -> None:
        """Resumes the current automation."""
        if not self.enabled:
            return
        if self.is_paused:
            assert self.automation is not None
            self.automation.resume()
            self.AUTOMATION_RESUMED.emit()
            rosys.notify('automation resumed')

    def stop(self, because: str) -> None:
        """Stops the current automation.

        You need to provide a cause which will be used as notification message.
        """
        if not self.is_stopped:
            assert self.automation is not None
            self.automation.stop()
            self.automation = None
            self.AUTOMATION_STOPPED.emit(because)
            rosys.notify(f'automation stopped because {because}')

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
        You need to provide a cause which will be used as notification message.
        """
        self.stop(because)
        self.enabled = False

    def set_default_automation(self, default_automation: Callable | None) -> None:
        """Sets the default automation.

        You can pass a function that returns a new coroutine on every call.
        """
        self.default_automation = default_automation

    def _handle_exception(self, e: Exception) -> None:
        self.stop(because='an exception occurred in an automation')
        self.AUTOMATION_FAILED.emit(f'automation aborted because of {e}')
        rosys.notify('automation failed', 'negative')
        if rosys.is_test:
            self.log.exception('automation failed')

    def _on_complete(self) -> None:
        self.AUTOMATION_COMPLETED.emit()
        rosys.notify('automation completed', 'positive')
