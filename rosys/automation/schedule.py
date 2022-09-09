from datetime import datetime, timedelta
from typing import Any, Callable, Optional

import rosys
import suntime
from nicegui import ui
from rosys import persistence

from .automator import Automator


class Schedule:

    def __init__(self,
                 automator: Automator, *,
                 on_enable: Optional[Callable] = None,
                 on_disable: Optional[Callable] = None,
                 location: Optional[tuple[float, float]] = None,
                 locations: Optional[dict[str, tuple[float, float]]] = None,
                 sunrise_offset: float = 0.0,
                 sunset_offset: float = 0.0,
                 ) -> None:
        '''Schedules automations according to a time plan.

        param on_enable: automation to execute when entering a time frame marked as active
        param on_disable: automation to execute when entering a time frame marked as inactive
        param location: optional geographic coordinates to activate the robot at daylight only
        param locations: optional dictionary with geographic coordinates to choose from
        param sunrise_offset: optional offset for activation after sunrise (minutes, default: 0.0)
        param sunset_offset: optional offset for deactivation after sunset (minutes, default: 0.0)
        '''
        self.automator = automator
        self.on_enable = on_enable
        self.on_disable = on_disable
        self.location = location
        self.locations = locations
        self.sunrise_offset = sunrise_offset
        self.sunset_offset = sunset_offset

        self.half_hours = [True] * 2 * 24 * 7

        self.enabled = False
        self.buttons: list[tuple(ui.button, ui.button, ui.button)] = []
        rosys.on_repeat(self.step, 1)

        self.needs_backup: bool = False
        persistence.register(self)

    def backup(self) -> dict:
        return {
            'location': self.location,
            'sunrise_offset': self.sunrise_offset,
            'sunset_offset': self.sunset_offset,
            'half_hours': self.half_hours,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.location = tuple(data.get('location')) if data.get('location') else None
        self.sunrise_offset = data.get('sunrise_offset')
        self.sunset_offset = data.get('sunset_offset')
        self.half_hours[:] = data.get('half_hours', True)
        self.update_ui()

    def invalidate(self) -> None:
        self.needs_backup = True

    def time_to_index(self, weekday: int, hour: int, minute: int) -> int:
        if not 0 <= weekday <= 6:
            raise ValueError(f'day of week must be between 0 and 6, not {weekday}')
        if not 0 <= minute <= 59:
            raise ValueError(f'minutes must be between 0 and 59, not {minute}')
        return weekday * 2 * 24 + hour * 2 + minute // 30

    def is_dark(self) -> bool:
        if not self.location:
            return False
        sun = suntime.Sun(lat=self.location[0], lon=self.location[1])
        start_time = sun.get_local_sunrise_time() + timedelta(minutes=self.sunrise_offset)
        stop_time = sun.get_local_sunset_time() + timedelta(minutes=self.sunset_offset)
        return not start_time < datetime.fromtimestamp(rosys.time()).astimezone() < stop_time

    def is_enabled(self, weekday: int, hour: int, minute: Optional[int] = None) -> bool:
        if minute is None:
            return self.half_hours[self.time_to_index(weekday, hour, 0)] \
                or self.half_hours[self.time_to_index(weekday, hour, 30)]
        else:
            return self.half_hours[self.time_to_index(weekday, hour, minute)]

    def fill(self, value: bool) -> None:
        for i in range(len(self.half_hours)):
            self.half_hours[i] = value

    def can_be_active(self) -> bool:
        time = datetime.fromtimestamp(rosys.time())
        return self.is_enabled(time.weekday(), time.hour, time.minute) and not self.is_dark()

    def step(self) -> None:
        if not self.enabled and self.can_be_active():
            if self.on_enable:
                self.automator.start(self.on_enable())
            self.enabled = True
        if self.enabled and not self.can_be_active():
            if self.on_disable:
                self.automator.start(self.on_disable())
            self.enabled = False

    def ui(self) -> ui.row:
        with ui.column().style(replace='gap: 0') as grid:
            if self.location and self.locations:
                with ui.row():
                    location_names = list(self.locations.keys())
                    ui.select(location_names,
                              label='Location',
                              value=[key for key in location_names if self.locations[key] == self.location][:1]) \
                        .classes('w-64')
                    ui.number('Sunrise offset', format='%.0f', on_change=self.invalidate) \
                        .bind_value(self, 'sunrise_offset').props('suffix=min')
                    ui.number('Sunset offset', format='%.0f', on_change=self.invalidate) \
                        .bind_value(self, 'sunset_offset').props('suffix=min')

            for d in range(7):
                with ui.row().style(f'padding: 0.3em; background-color: {"#ccc" if d > 4 else "#fff"}', replace='gap: 0.3em'):
                    for h in range(24):
                        with ui.column().style(replace='gap: 0.1em'):
                            hour = ui.button(f'{h:02d}', on_click=lambda _, d=d, h=h: self._toggle(d, h)) \
                                .props('unelevated dense')
                            with ui.row().style(replace='gap: 0.1em'):
                                first_half = ui.button(on_click=lambda _, d=d, h=h: self._toggle(d, h, 0)) \
                                    .props('unelevated dense').style('height: 1em; width: 0.8em')
                                second_half = ui.button(on_click=lambda _, d=d, h=h: self._toggle(d, h, 30)) \
                                    .props('unelevated dense').style('height: 1em; width: 0.8em')
                        self.buttons.append((hour, first_half, second_half))

        self.update_ui()
        return grid

    def update_ui(self) -> None:
        for d in range(7):
            for h in range(24):
                buttons = self.buttons[d * 24 + h]
                buttons[0].classes(replace='bg-positive' if self.is_enabled(d, h) else 'bg-negative')
                buttons[1].classes(replace='bg-positive' if self.is_enabled(d, h, 0) else 'bg-negative')
                buttons[2].classes(replace='bg-positive' if self.is_enabled(d, h, 30) else 'bg-negative')

    def _toggle(self, weekday: int, hour: int, minute: Optional[int] = None) -> None:
        new_value = not self.is_enabled(weekday, hour, minute)
        if minute is None:
            self.half_hours[self.time_to_index(weekday, hour, 0)] = new_value
            self.half_hours[self.time_to_index(weekday, hour, 30)] = new_value
        else:
            self.half_hours[self.time_to_index(weekday, hour, minute)] = new_value
        self.needs_backup = True
        self.update_ui()
