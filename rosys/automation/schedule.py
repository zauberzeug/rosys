from datetime import datetime
from typing import Callable, Optional

import rosys
from nicegui import ui

from .automator import Automator


class Plan:

    def __init__(self) -> None:
        self.half_hours: list[bool] = [True] * 24 * 2

    def time_to_index(self, hour: int, minute: int) -> int:
        if not 0 <= minute <= 59:
            raise ValueError(f'minutes must be between 0 and 59, not {minute}')
        return hour * 2 + minute // 30

    def is_enabled(self, hour: int, minute: Optional[int] = None) -> bool:
        if minute is None:
            return self.half_hours[self.time_to_index(hour, 0)] or self.half_hours[self.time_to_index(hour, 30)]
        else:
            return self.half_hours[self.time_to_index(hour, minute)]

    def toggle(self, hour: int, minute: Optional[int] = None) -> bool:
        new_value = not self.is_enabled(hour, minute)
        if minute is None:
            self.half_hours[self.time_to_index(hour, 0)] = new_value
            self.half_hours[self.time_to_index(hour, 30)] = new_value
        else:
            self.half_hours[self.time_to_index(hour, minute)] = new_value

    def disable_all(self) -> None:
        for i in range(len(self.half_hours)):
            self.half_hours[i] = False

    def enable_all(self) -> None:
        for i in range(len(self.half_hours)):
            self.half_hours[i] = True


class Schedule:

    def __init__(self, automator: Automator, *,
                 on_enable: Optional[Callable] = None,
                 on_disable: Optional[Callable] = None) -> None:
        '''Schedules automations according to a time plan.

        param on_enable: automation to execute when entering a time frame marked as active
        param on_disable: automation to execute when entering a time frame marked as inactive'''

        self.automator = automator
        self.on_enable = on_enable
        self.on_disable = on_disable

        self.plan = Plan()
        self.enabled = False
        self.buttons: list[tuple(ui.button, ui.button, ui.button)] = []
        rosys.on_repeat(self.step, 1)

    def step(self) -> None:
        time = datetime.fromtimestamp(rosys.time())
        if not self.enabled and self.plan.is_enabled(time.hour, time.minute) and self.on_enable:
            self.automator.start(self.on_enable())
            self.enabled = True
        if self.enabled and not self.plan.is_enabled(time.hour, time.minute) and self.on_disable:
            self.automator.start(self.on_disable())
            self.enabled = False

    def ui(self) -> ui.row:
        with ui.row().style(replace='gap: 0.3em') as row:
            for i in range(24):
                with ui.column().style(replace='gap: 0.3em'):
                    hour = ui.button(f'{i:02d}', on_click=lambda _, i=i: self.toggle(i)).props('unelevated dense')
                    with ui.row().style(replace='gap: 0.1em'):
                        first_half = ui.button(on_click=lambda _, i=i: self.toggle(i, 0)).props('unelevated dense') \
                            .style('width:0.8em')
                        second_half = ui.button(on_click=lambda _, i=i: self.toggle(i, 30)).props('unelevated dense') \
                            .style('width:0.8em')
                self.buttons.append((hour, first_half, second_half))
        self.update_ui()
        return row

    def update_ui(self) -> None:
        for i, hour_buttons in enumerate(self.buttons):
            hour_buttons[0].classes(replace='bg-positive' if self.plan.is_enabled(i) else 'bg-negative')
            hour_buttons[1].classes(replace='bg-positive' if self.plan.is_enabled(i, 0) else 'bg-negative')
            hour_buttons[2].classes(replace='bg-positive' if self.plan.is_enabled(i, 30) else 'bg-negative')

    def toggle(self, hour: int, minute: Optional[int] = None) -> None:
        self.plan.toggle(hour, minute)
        self.update_ui()
