import asyncio
import logging

from nicegui import ui

from .. import rosys


class AsyncioWarnings:

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.asyncio_warnings')
        rosys.on_startup(lambda: self.set_threshold(0.05))

    def activate(self) -> None:
        """Produce warnings for coroutines which take too long on the main loop and hence clog the event loop.

        Beware: This slows everything down. Do not use in production or while profiling!
        """
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(True)
            self.log.warning('activated asyncio warnings; do not use in production or while profiling)')
        except Exception:
            self.log.exception('could not activate asyncio warnings')

    def deactivate(self) -> None:
        """Stop the warnings for slow coroutines."""
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(False)
        except Exception:
            self.log.exception('could not deactivate asyncio warnings')

    def toggle(self) -> None:
        loop = asyncio.get_running_loop()
        if loop.get_debug():
            self.deactivate()
        else:
            self.activate()

    def set_threshold(self, seconds: float) -> None:
        """Sets the threshold in seconds after which a coroutine is considered slow."""
        loop = asyncio.get_running_loop()
        loop.slow_callback_duration = seconds

    def ui(self) -> ui.element:
        with ui.row().classes('items-end') as content:
            loop = asyncio.get_running_loop()
            ui.checkbox(on_change=self.toggle)
            ui.number('coro limit',
                      value=loop.slow_callback_duration, format='%.3f',
                      on_change=lambda e: self.set_threshold(e.value)) \
                .props('suffix=s').classes('w-16')
        return content
