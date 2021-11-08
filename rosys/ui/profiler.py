from nicegui.ui import Ui
import logging
import socket
import os.path
from pathlib import Path
import asyncio
import yappi
from tabulate import tabulate

log = logging.getLogger('rosys.profiler')


def create_profiler(ui: Ui):

    def filter(stat: yappi.YFuncStat):
        if 'python' in stat.module:
            return False
        return True

    def toggle():
        if yappi.is_running():
            table = []
            for stat in yappi.get_func_stats(filter_callback=filter):
                row = [str(v) for v in [stat.full_name, stat.ttot,  stat.tsub, stat.tavg, stat.ncall]]
                table.append(row)
            print(tabulate(table, headers=['function', 'total', 'excl. sub', 'avg', 'ncall']), flush=True)
            yappi.get_thread_stats().print_all()
            yappi.stop()
            ui.notify('stop profiling')
            profile_button.props(replace='icon=play_arrow')
        else:
            profile_button.props(replace='icon=stop')
            yappi.clear_stats()
            yappi.start()
            ui.notify('start profiling')
        return False  # do not refresh UI

    profile_button = ui.button('Profiler', on_click=toggle).props('icon=play_arrow')
