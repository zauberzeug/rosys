import time

import yappi
from nicegui import ui
from tabulate import tabulate

from .. import rosys
from .profiling import profile


class ProfileButton(ui.button):
    """The profile button allows starting and stopping a profiling session.

    Use the `profiling.profile` decorator for including functions or methods in the analysis.
    The results are shown on the console.
    """

    def __init__(self) -> None:
        super().__init__('Profiler', on_click=self.toggle)
        self.props('icon=play_arrow')
        self.tooltip('run profiling')

    async def start(self, duration: float = 10.0) -> None:
        ui.notify('start profiling')
        self.props('icon=stop')
        yappi.clear_stats()
        yappi.start()
        profile.start()
        t = time.time()
        while yappi.is_running() and time.time() < t + duration:
            await rosys.sleep(0.1)
        self.stop()

    def stop(self) -> None:
        ui.notify('stop profiling')
        self.props('icon=play_arrow')
        profile.stop(print_stats=False)
        yappi.stop()
        table = [
            [str(v) for v in [stat.full_name, stat.ttot, stat.tsub, stat.tavg, stat.ncall]]
            for stat in yappi.get_func_stats()
            if 'python' not in stat.module
        ]
        output = tabulate(table[:15], headers=['function', 'total', 'excl. sub', 'avg', 'ncall'], floatfmt='.4f')
        print(output, flush=True)
        yappi.get_thread_stats().print_all()  # pylint: disable=no-member
        profile.print()

    async def toggle(self) -> None:
        if yappi.is_running():
            self.stop()
        else:
            await self.start()
