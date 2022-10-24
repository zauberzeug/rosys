import asyncio
import logging

import rosys
from nicegui import ui


class AsyncioWarnings():

    def __init__(self):
        self.log = logging.getLogger('rosys.asyncio_warnings')
        rosys.on_startup(lambda: self.set_treshold(0.05))

    def activate(self) -> None:
        '''Produce warnings for coroutines which take too long on the main loop and hence clog the event loop.

        Beware: this is slowing everything down. Do not use in production or while profiling!
        '''
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(True)
            self.log.warning('activated asyncio warnings; do not use in production or while profiling)')
        except:
            self.log.exception('could not activate asyncio warnings')

    def deactivate(self) -> None:
        '''Stop the warnings for slow coroutines.'''
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(False)
        except:
            self.log.exception('could not deactivate asyncio warnings')

    def toggle(self) -> None:
        loop = asyncio.get_running_loop()
        if loop.get_debug():
            self.deactivate()
        else:
            self.activate()

    def set_treshold(self, seconds: float) -> None:
        '''Sets the treshold in seconds after which a coroutine is considered slow.'''
        loop = asyncio.get_running_loop()
        loop.slow_callback_duration = seconds

    def ui(self):
        ui.switch('asyncio warnings', on_change=self.toggle)
        loop = asyncio.get_running_loop()
        ui.number('asyncio slowness-threshold', value=loop.slow_callback_duration,
                  format='%.3f', on_change=lambda e: self.set_treshold(e.value))
