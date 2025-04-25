import functools
import logging
from collections.abc import Callable
from datetime import datetime, timedelta
from typing import Any, cast

import suntime
from nicegui import ui

from .. import persistence, rosys
from ..event import Event
from .automator import Automator


class Translator:
    Location = 'Location'
    Sunrise_offset = 'Sunrise offset'
    Sunset_offset = 'Sunset offset'
    min = 'min'
    enabled = 'enabled'
    Mon = 'Mon'
    Tue = 'Tue'
    Wed = 'Wed'
    Thu = 'Thu'
    Fri = 'Fri'
    Sat = 'Sat'
    Sun = 'Sun'


class Schedule(persistence.Persistable):

    def __init__(self, *,
                 automator: Automator | None = None,
                 on_activate: Callable | None = None,
                 on_deactivate: Callable | None = None,
                 location: tuple[float, float] | None = None,
                 locations: dict[tuple[float, float], str] | None = None,
                 sunrise_offset: float = 0.0,
                 sunset_offset: float = 0.0,
                 is_enabled: bool = False,
                 ) -> None:
        """Schedules automations according to a time plan.

        :param automator: optional automator to use for executing the automations
        :param on_activate: automation to execute when entering an active period
        :param on_deactivate: automation to execute when entering an inactive period
        :param location: optional geographic coordinates to activate the robot at daylight only
        :param locations: optional dictionary with geographic coordinates to choose from
        :param sunrise_offset: optional offset for activation after sunrise (minutes, default: 0.0)
        :param sunset_offset: optional offset for deactivation after sunset (minutes, default: 0.0)
        :param is_enabled: whether the schedule is enabled (default: False)
        """
        super().__init__()

        self.log = logging.getLogger('rosys.schedule')

        self.SCHEDULE_CHANGED = Event[[]]()
        """the schedule has changed"""

        self.automator = automator
        self.translator: Translator = rosys.translator or Translator()
        self.on_activate = on_activate
        self.on_deactivate = on_deactivate
        self.location = location
        self.locations = locations
        self.sunrise_offset = sunrise_offset
        self.sunset_offset = sunset_offset
        self.is_enabled = is_enabled

        self.half_hours = [True] * 2 * 24 * 7

        self._is_active = False
        rosys.on_repeat(self.step, 1)

    def backup_to_dict(self) -> dict:
        return {
            'location': self.location,
            'sunrise_offset': self.sunrise_offset,
            'sunset_offset': self.sunset_offset,
            'is_enabled': self.is_enabled,
            'is_active': self._is_active,
            'half_hours': self.half_hours,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.location = (data['location'][0], data['location'][1]) if data.get('location') else None
        self.sunrise_offset = cast(float, data.get('sunrise_offset'))
        self.sunset_offset = cast(float, data.get('sunset_offset'))
        self.is_enabled = cast(bool, data.get('is_enabled', False))
        self._is_active = cast(bool, data.get('is_active', False))
        self.half_hours[:] = cast(list[bool], data['half_hours']) if 'half_hours' in data else [True] * 2 * 24 * 7
        self.SCHEDULE_CHANGED.emit()

    def invalidate(self) -> None:
        self.request_backup()
        self.SCHEDULE_CHANGED.emit()

    def time_to_index(self, weekday: int, hour: int, minute: int) -> int:
        if not 0 <= weekday <= 6:
            raise ValueError(f'day of week must be between 0 and 6, not {weekday}')
        if not 0 <= minute <= 60:
            raise ValueError(f'minutes must be between 0 and 60, not {minute}')
        return weekday * 2 * 24 + hour * 2 + minute // 30

    def is_dark(self, hour: int, minute: int) -> bool:
        sun_start_hour = _get_sun_start_hour(self.location, self.sunrise_offset)
        sun_stop_hour = _get_sun_stop_hour(self.location, self.sunset_offset)
        return not sun_start_hour < hour + minute / 60 < sun_stop_hour

    def is_planned(self, weekday: int, hour: int, minute: int | None = None) -> bool:
        if minute is None:
            return self.half_hours[self.time_to_index(weekday, hour, 0)] \
                or self.half_hours[self.time_to_index(weekday, hour, 30)]
        return self.half_hours[self.time_to_index(weekday, hour, minute)]

    def fill(self, value: bool) -> None:
        for i, _ in enumerate(self.half_hours):
            self.half_hours[i] = value

    def can_be_active(self) -> bool:
        now = datetime.fromtimestamp(rosys.time())
        return self.is_planned(now.weekday(), now.hour, now.minute) and not self.is_dark(now.hour, now.minute)

    def step(self) -> None:
        if not self.is_enabled:
            return
        if not self._is_active and self.can_be_active():
            self.log.info('activate')
            if self.on_activate:
                self.log.info('start automation')
                if self.automator:
                    self.automator.start(self.on_activate(), paused=self.automator.is_paused)
                else:
                    self.on_activate()
            self._is_active = True
        if self._is_active and not self.can_be_active():
            self.log.info('deactivate')
            if self.on_deactivate:
                self.log.info('start automation')
                if self.automator:
                    self.automator.start(self.on_deactivate(), paused=self.automator.is_paused)
                else:
                    self.on_deactivate()
            self._is_active = False

    def ui(self) -> ui.column:
        def update() -> None:
            def color(weekday: int, hour: int, minutes: list[int]) -> str:
                # https://maketintsandshades.com/#21ba45,c10015
                if all(self.is_dark(hour, minute) for minute in minutes):
                    return '#d3f1da' if self.is_planned(weekday, hour, minutes[0]) else '#f3ccd0'
                dt = datetime.fromtimestamp(rosys.time())
                is_now = dt.weekday() == weekday and dt.hour == hour
                positive = '#147029' if is_now else '#21ba45' if d < 5 else '#1a9537'
                negative = '#74000d' if is_now else '#c10015' if d < 5 else '#9a0011'
                return positive if self.is_planned(weekday, hour, minutes[0]) else negative
            if not buttons:
                return
            for d in range(7):
                for h in range(24):
                    b0, b1, b2 = buttons[d * 24 + h]
                    b0.classes(replace=f'!bg-[{color(d, h, [0, 30, 60])}]')
                    b1.classes(replace=f'!bg-[{color(d, h, [0, 30])}]')
                    b2.classes(replace=f'!bg-[{color(d, h, [30, 60])}]')

        t = self.translator
        buttons: list[tuple[ui.button, ui.button, ui.button]] = []
        with ui.column() as grid:
            with ui.row().classes('fit items-center justify-between'):
                ui.switch(t.enabled).bind_value(self, 'is_enabled')
                if self.locations:
                    ui.select(self.locations, label=t.Location, on_change=lambda e: self.set_location(e.value)) \
                        .bind_value(self, 'location').style('width: 21em')
                    ui.number(t.Sunrise_offset, format='%.0f', on_change=self.invalidate, suffix=t.min) \
                        .bind_value(self, 'sunrise_offset').style('width:100px')
                    ui.number(t.Sunset_offset, format='%.0f', on_change=self.invalidate, suffix=t.min) \
                        .bind_value(self, 'sunset_offset').style('width:100px')
            with ui.column().style('gap: 0.3em'):
                for d, day in enumerate([t.Mon, t.Tue, t.Wed, t.Thu, t.Fri, t.Sat, t.Sun]):
                    with ui.row().style('gap: 0.3em'):
                        ui.label(day).classes('mt-2').style('width: 2em')
                        for h in range(24):
                            with ui.column().style('gap: 0.1em'):
                                hour = ui.button(f'{h:02d}', on_click=lambda _, d=d, h=h: self._toggle(d, h)) \
                                    .props('unelevated dense')
                                with ui.row().style('gap: 0.1em'):
                                    first_half = ui.button(on_click=lambda _, d=d, h=h: self._toggle(d, h, 0)) \
                                        .props('unelevated dense').style('height: 1em; width: 0.8em')
                                    second_half = ui.button(on_click=lambda _, d=d, h=h: self._toggle(d, h, 30)) \
                                        .props('unelevated dense').style('height: 1em; width: 0.8em')
                            buttons.append((hour, first_half, second_half))

        ui.timer(60, update)  # NOTE: to update the "now" indicator
        self.SCHEDULE_CHANGED.register_ui(update)
        return grid

    def set_location(self, location: tuple[float, float]) -> None:
        self.location = location
        self.invalidate()

    def _toggle(self, weekday: int, hour: int, minute: int | None = None) -> None:
        new_value = not self.is_planned(weekday, hour, minute)
        if minute is None:
            self.half_hours[self.time_to_index(weekday, hour, 0)] = new_value
            self.half_hours[self.time_to_index(weekday, hour, 30)] = new_value
        else:
            self.half_hours[self.time_to_index(weekday, hour, minute)] = new_value
        self.invalidate()


@functools.lru_cache(maxsize=100)
def _get_sun_start_hour(location: tuple[float, float], offset: float) -> float:
    if not location:
        return 0.0
    sun = suntime.Sun(lat=location[0], lon=location[1])
    start_time = sun.get_sunrise_time(datetime.now()).astimezone() + timedelta(minutes=offset)
    return start_time.hour + start_time.minute / 60 + start_time.second / 60 / 60


@functools.lru_cache(maxsize=100)
def _get_sun_stop_hour(location, offset) -> float:
    if not location:
        return 24.0
    sun = suntime.Sun(lat=location[0], lon=location[1])
    stop_time = sun.get_sunset_time(datetime.now()).astimezone() + timedelta(minutes=offset)
    return stop_time.hour + stop_time.minute / 60 + stop_time.second / 60 / 60
