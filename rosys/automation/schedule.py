from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Optional

import rosys
import suntime
from nicegui import ui
from rosys import persistence

from .automator import Automator


@dataclass(slots=True, kw_only=True)
class Plan:
    half_hours: list[bool] = field(default_factory=lambda: [True] * 2 * 24 * 7)
    location: Optional[tuple[float, float]] = None

    def time_to_index(self, weekday: int, hour: int, minute: int) -> int:
        if not 0 <= weekday <= 6:
            raise ValueError(f'day of week must be between 0 and 6, not {weekday}')
        if not 0 <= minute <= 59:
            raise ValueError(f'minutes must be between 0 and 59, not {minute}')
        return weekday * 2 * 24 + hour * 2 + minute // 30

    def is_enabled(self, weekday: int, hour: int, minute: Optional[int] = None) -> bool:
        if self.location:
            sun = suntime.Sun(lat=self.location[0], lon=self.location[1])
            dt = datetime.now().astimezone().replace(hour=hour, minute=minute or 0)
            if not sun.get_local_sunrise_time() < dt < sun.get_local_sunset_time():
                return False
        if minute is None:
            return self.half_hours[self.time_to_index(weekday, hour, 0)] \
                or self.half_hours[self.time_to_index(weekday, hour, 30)]
        else:
            return self.half_hours[self.time_to_index(weekday, hour, minute)]

    def toggle(self, weekday: int, hour: int, minute: Optional[int] = None) -> bool:
        new_value = not self.is_enabled(weekday, hour, minute)
        if minute is None:
            self.half_hours[self.time_to_index(weekday, hour, 0)] = new_value
            self.half_hours[self.time_to_index(weekday, hour, 30)] = new_value
        else:
            self.half_hours[self.time_to_index(weekday, hour, minute)] = new_value

    def disable_all(self) -> None:
        for i in range(len(self.half_hours)):
            self.half_hours[i] = False

    def enable_all(self) -> None:
        for i in range(len(self.half_hours)):
            self.half_hours[i] = True


class Schedule:

    def __init__(self, automator: Automator, *,
                 on_enable: Optional[Callable] = None,
                 on_disable: Optional[Callable] = None,
                 location: Optional[tuple[float, float]] = None) -> None:
        '''Schedules automations according to a time plan.

        param on_enable: automation to execute when entering a time frame marked as active
        param on_disable: automation to execute when entering a time frame marked as inactive
        param location: optional geographic coordinates to activate the robot at daylight only'''

        self.automator = automator
        self.on_enable = on_enable
        self.on_disable = on_disable

        self.plan = Plan(location=location)
        self.enabled = False
        self.buttons: list[tuple(ui.button, ui.button, ui.button)] = []
        rosys.on_repeat(self.step, 1)

        self.needs_backup: bool = False
        persistence.register(self)

    def backup(self) -> dict:
        return {'half_hours': self.plan.half_hours}

    def restore(self, data: dict[str, Any]) -> None:
        self.plan.half_hours[:] = data.get('half_hours', True)
        self.update_ui()

    def step(self) -> None:
        time = datetime.fromtimestamp(rosys.time())
        if not self.enabled and self.plan.is_enabled(time.weekday(), time.hour, time.minute) and self.on_enable:
            self.automator.start(self.on_enable())
            self.enabled = True
        if self.enabled and not self.plan.is_enabled(time.weekday(), time.hour, time.minute) and self.on_disable:
            self.automator.start(self.on_disable())
            self.enabled = False

    def ui(self) -> ui.row:
        with ui.column().style(replace='gap: 0') as grid:
            for d in range(7):
                with ui.row().style(f'padding: 0.3em; background-color: {"#ccc" if d > 4 else "#fff"}', replace='gap: 0.3em'):
                    for h in range(24):
                        with ui.column().style(replace='gap: 0.1em'):
                            hour = ui.button(f'{h:02d}', on_click=lambda _, d=d, h=h: self.toggle(d, h)) \
                                .props('unelevated dense')
                            with ui.row().style(replace='gap: 0.1em'):
                                first_half = ui.button(on_click=lambda _, d=d, h=h: self.toggle(d, h, 0)) \
                                    .props('unelevated dense').style('height: 1em; width: 0.8em')
                                second_half = ui.button(on_click=lambda _, d=d, h=h: self.toggle(d, h, 30)) \
                                    .props('unelevated dense').style('height: 1em; width: 0.8em')
                        self.buttons.append((hour, first_half, second_half))
        self.update_ui()
        return grid

    def update_ui(self) -> None:
        for d in range(7):
            for h in range(24):
                buttons = self.buttons[d * 24 + h]
                buttons[0].classes(replace='bg-positive' if self.plan.is_enabled(d, h) else 'bg-negative')
                buttons[1].classes(replace='bg-positive' if self.plan.is_enabled(d, h, 0) else 'bg-negative')
                buttons[2].classes(replace='bg-positive' if self.plan.is_enabled(d, h, 30) else 'bg-negative')

    def toggle(self, weekday: int, hour: int, minute: Optional[int] = None) -> None:
        self.plan.toggle(weekday, hour, minute)
        self.needs_backup = True
        self.update_ui()
