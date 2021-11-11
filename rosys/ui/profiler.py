from nicegui.ui import Ui
import logging
import yappi
from tabulate import tabulate

log = logging.getLogger('rosys.profiler')


def create_profiler(ui: Ui):

    def toggle() -> bool:
        if yappi.is_running():
            ui.notify('stop profiling')
            profile_button.props(replace='icon=play_arrow')
            yappi.stop()
            table = [
                [str(v) for v in [stat.full_name, stat.ttot, stat.tsub, stat.tavg, stat.ncall]]
                for stat in yappi.get_func_stats()
                if 'python' not in stat.module
            ]
            output = tabulate(
                table,
                headers=['function', 'total', 'excl. sub', 'avg', 'ncall'],
                floatfmt=".4f"
            )
            print(output, flush=True)
            yappi.get_thread_stats().print_all()
        else:
            ui.notify('start profiling')
            profile_button.props(replace='icon=stop')
            yappi.clear_stats()
            yappi.start()
        return False  # do not refresh UI

    profile_button = ui.button('Profiler', on_click=toggle).props('icon=play_arrow')
