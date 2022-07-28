import logging
import time

import yappi
from nicegui import ui
from tabulate import tabulate

from ..profiling import profile
import rosys

log = logging.getLogger('rosys.profiler')


def profile_button() -> ui.button:

    async def start(duration: float = 10.0):
        ui.notify('start profiling')
        button.props(replace='icon=stop')
        yappi.clear_stats()
        yappi.start()
        profile.start()
        t = time.time()
        while yappi.is_running() and time.time() < t + duration:
            await rosys.sleep(0.1)
        stop()

    def stop():
        ui.notify('stop profiling')
        button.props(replace='icon=play_arrow')
        profile.stop(print=False)
        yappi.stop()
        table = [
            [str(v) for v in [stat.full_name, stat.ttot, stat.tsub, stat.tavg, stat.ncall]]
            for stat in yappi.get_func_stats()
            if 'python' not in stat.module
        ]
        output = tabulate(
            table[:15],
            headers=['function', 'total', 'excl. sub', 'avg', 'ncall'],
            floatfmt='.4f'
        )
        print(output, flush=True)
        yappi.get_thread_stats().print_all()
        profile.print()

    async def toggle() -> None:
        if yappi.is_running():
            stop()
        else:
            await start()

    button = ui.button('Profiler', on_click=toggle).props('icon=play_arrow').tooltip('run profiling')
    return button
