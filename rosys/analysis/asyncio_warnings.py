import asyncio
import logging

from humanize import activate, deactivate
from nicegui import ui


class AsyncioWarnings():

    def __init__(self):
        self.log = logging.getLogger('rosys.asyncio_warnings')
        self.debugging = False

    def activate(self) -> None:
        '''Produce warnings for coroutines which take too long on the main loop and hence clog the event loop.

        Beware: this is slowing everything down. Do not use in production or while profiling!
        '''
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(True)
            loop.slow_callback_duration = 0.05
            self.debugging = True
            self.log.warning('activated asyncio warnings; do not use in production or while profiling)')
        except:
            self.log.exception('could not activate asyncio warnings')

    def deactivate(self) -> None:
        '''Stop the warnings for slow coroutines.'''
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(False)
            self.debugging = False
        except:
            self.log.exception('could not deactivate asyncio warnings')

    def toggle(self) -> None:
        if self.debugging:
            deactivate()
        else:
            activate()

    def ui(self):
        ui.switch('asyncio warnings', on_change=self.toggle)
