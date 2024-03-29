#!/usr/bin/env python3
import rosys
from nicegui import ui
from rosys.analysis import profile_button, profiling


@profiling.profile
def compute() -> None:
    s = 0
    for i in range(1_000_000):
        s += i**2
    ui.notify(s)


rosys.on_repeat(compute, 1.0)
profile_button()

ui.run()
