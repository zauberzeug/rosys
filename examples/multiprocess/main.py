#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui
from multiprocessing import Queue
from cpu_intensive_process import CpuIntensiveProcess

runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)

latest_result = Queue()
process = CpuIntensiveProcess(latest_result)

ui.on_startup(process.start)
ui.on_shutdown(process.stop)

label = ui.label()
ui.timer(1.0, lambda: label.set_text(f'{latest_result.get()=}'))

ui.run(title="Multiprocess Example", port=8080)
