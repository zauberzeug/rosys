from datetime import datetime
from typing import Awaitable, Optional

from nicegui import ui

from ..runtime import runtime
from . import Automator


class Plan:

    def __init__(self) -> None:
        self.half_hours: list[bool] = [True for _ in range(24*2)]

    def is_enabled(self, hour: int, minute: Optional[int] = None) -> bool:
        index = hour * 2
        first_half, second_half = self.half_hours[index:index+2]
        if minute is None:
            return first_half or second_half
        if minute < 0 or minute > 59:
            raise ValueError(f'minutes must be between 0 and 59, not {minute}')
        if minute < 30 and first_half:
            return True
        if minute >= 30 and second_half:
            return True
        return False

    def toggle(self, hour: int, minute: Optional[int] = None) -> bool:
        index = hour * 2
        if minute is None:
            self.half_hours[index] = self.half_hours[index+1] = not self.is_enabled(hour)
            return
        if minute < 0 or minute > 59:
            raise ValueError(f'minutes must be between 0 and 59, not {minute}')
        if minute < 30:
            self.half_hours[index] = not self.is_enabled(hour, minute)
        if minute >= 30:
            self.half_hours[index+1] = not self.is_enabled(hour, minute)

    def disable_all(self):
        for i in range(len(self.half_hours)):
            self.half_hours[i] = False

    def enable_all(self):
        for i in range(len(self.half_hours)):
            self.half_hours[i] = True


class TimeSchedule:

    def __init__(self,
                 automator: Automator, *,
                 on_enable: Optional[Awaitable] = None,
                 on_disable: Optional[Awaitable] = None,
                 ) -> None:
        '''Schedules automations according to a time plan.

        param on_enable: automation to execute when entering an time frame marked as active
        param on_disable: automation to execute when entering an time frame marked as inactive'''

        self.automator: Automator = automator
        self.on_enable = on_enable
        self.on_disable = on_disable
        self.plan: Plan = Plan()
        self.enabled: bool = False
        self.buttons: list[tuple(ui.button, ui.button, ui.button)] = []
        runtime.on_repeat(self.step, 1)

    def step(self):
        time = datetime.fromtimestamp(runtime.time)
        if not self.enabled and \
                self.plan.is_enabled(time.hour, time.minute) and \
                self.on_enable is not None:
            self.automator.start(self.on_enable())
            self.enabled = True
        if self.enabled and \
                not self.plan.is_enabled(time.hour, time.minute) and \
                self.on_disable is not None:
            self.automator.start(self.on_disable())
            self.enabled = False

    def ui(self):
        with ui.row().style(replace='gap: 0.3em'):
            for i in range(24):
                with ui.column().style(replace='gap: 0.3em'):
                    hour = ui.button(str(i).zfill(2), on_click=lambda _, i=i: self.toggle(i)).props('unelevated dense')
                    w = 0.8
                    with ui.row().style(replace='gap: 0.1em'):
                        first_half = ui.button('', on_click=lambda _, i=i: self.toggle(i, 0)). \
                            props('unelevated dense').style(f'width:{w}em')
                        second_half = ui.button('', on_click=lambda _, i=i: self.toggle(i, 30)). \
                            props('unelevated dense').style(f'width:{w}em')
                self.buttons.append((hour, first_half, second_half))
        self.update_ui()

    def update_ui(self):
        for i, hour_buttons in enumerate(self.buttons):
            hour_buttons[0].classes(replace='bg-positive' if self.plan.is_enabled(i) else 'bg-negative')
            hour_buttons[1].classes(replace='bg-positive' if self.plan.is_enabled(i, 0) else 'bg-negative')
            hour_buttons[2].classes(replace='bg-positive' if self.plan.is_enabled(i, 30) else 'bg-negative')

    def toggle(self, hour: int, minute: Optional[int] = None):
        self.plan.toggle(hour, minute)
        self.update_ui()
