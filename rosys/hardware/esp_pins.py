import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING

from nicegui import ui
from nicegui.elements.mixins.value_element import ValueElement

from .. import rosys

if TYPE_CHECKING:
    from rosys.hardware.robot_brain import RobotBrain


class StatusBulb(ValueElement):
    def __init__(self, value: bool = False) -> None:
        super().__init__(value=value, on_value_change=self.on_change, tag='span')
        self.style('height: 15px; width: 15px; margin: auto; border-radius: 50%')
        self.on_change()

    def on_change(self) -> None:
        self.style('background: radial-gradient(circle at 5px 5px, #5898d4, #4682B4);' if self.value
                   else 'background: radial-gradient(circle at 5px 5px, #D3D3D3, #A9A9A9);')


@dataclass
class GPIOPin:
    gpio: int
    level: bool
    is_input: bool
    is_output: bool
    open_drain: bool
    is_pullup: bool
    is_pulldown: bool
    drive_strength: int
    sleep_sel: int


class ESPPins:
    """Monitor and control ESP32 GPIO pins."""
    TIME_BETWEEN_REQUESTS = 0.05

    def __init__(self, name: str, robot_brain: 'RobotBrain', update_interval: float = 2.0) -> None:
        self.log = logging.getLogger('rosys.esp_pins')
        self.name = name
        self.robot_brain = robot_brain
        self._pin_numbers: list[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                        15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 37, 38, 39]
        assert update_interval > self.TIME_BETWEEN_REQUESTS * len(self._pin_numbers)
        self._gpio_states: dict[int, GPIOPin] = {pin: GPIOPin(
            pin, False, False, False, False, False, False, False, False) for pin in self._pin_numbers}
        self._auto_update: bool = False
        robot_brain.LINE_RECEIVED.register(self.parse)
        rosys.on_repeat(self.run_auto_update, update_interval)

    def parse(self, line: str) -> None:
        # GPIO_Status[3]| Level: 1| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| DriveStrength: 2| SleepSel: 0@7c
        # p0: GPIO_Status[0]| Level: 1| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| DriveStrength: 2| SleepSel: 0
        if 'GPIO_Status' not in line:
            return
        if self.name == 'core' and not line.startswith('GPIO_Status'):
            return
        if self.name != 'core' and not line.startswith(self.name):
            return
        parts = [part.strip() for part in line.split('|')]
        status_dict = {}
        for part in parts[1:]:
            key, value = part.split(': ')
            status_dict[key.strip()] = int(value.strip())

        gpio_str = parts[0].split('[')[1].split(']')[0]
        pin = int(gpio_str)
        self._gpio_states[pin].level = bool(status_dict.get('Level', 0))
        self._gpio_states[pin].is_input = bool(status_dict.get('InputEn', 0))
        self._gpio_states[pin].is_output = bool(status_dict.get('OutputEn', 0))
        self._gpio_states[pin].open_drain = bool(status_dict.get('OpenDrain', 0))
        self._gpio_states[pin].is_pullup = bool(status_dict.get('Pullup', 0))
        self._gpio_states[pin].is_pulldown = bool(status_dict.get('Pulldown', 0))
        self._gpio_states[pin].drive_strength = int(status_dict.get('DriveStrength', 0))
        self._gpio_states[pin].sleep_sel = int(status_dict.get('SleepSel', 0))

    async def update_pin(self, pin_number: int) -> None:
        await self.robot_brain.send(f'{self.name}.get_pin_status({pin_number})')
        await rosys.sleep(self.TIME_BETWEEN_REQUESTS)

    async def update_all(self) -> None:
        for pin_number in self._pin_numbers:
            await self.update_pin(pin_number)

    async def run_auto_update(self) -> None:
        if not self._auto_update:
            return
        await self.update_all()

    async def set_pin_level(self, pin_number: int, level: bool) -> None:
        await self.robot_brain.send(f'{self.name}.set_pin_level({pin_number}, {1 if level else 0})')
        await rosys.sleep(self.TIME_BETWEEN_REQUESTS)

    def developer_ui(self):
        with ui.column():
            with ui.row().classes('w-full'):
                ui.markdown(f'**ESP: {self.name}**')
                ui.space()
                ui.button(icon='refresh', on_click=self.update_all).props(
                    'flat round dense').tooltip('Update once')
                ui.switch().bind_value(self, '_auto_update').tooltip('Activate auto update')
            ui.separator()
            with ui.grid(columns=5):
                for gpio_state in self._gpio_states.values():
                    with ui.row():
                        ui.label(f'GPIO {str(gpio_state.gpio).zfill(2)}')
                        with StatusBulb().bind_value_from(gpio_state, 'level'):
                            with ui.context_menu().props('auto-close=false') as context_menu:
                                with ui.card():
                                    with ui.row().classes('w-full'):
                                        ui.label(f'GPIO {gpio_state.gpio}')
                                        ui.space()
                                        ui.button(icon='refresh', on_click=lambda pin_number=gpio_state.gpio: self.update_pin(pin_number)).props(
                                            'size=sm flat dense').tooltip('Update gpio state')
                                        ui.button(icon='check_circle', on_click=lambda pin_number=gpio_state.gpio: self.set_pin_level(pin_number, True)).props(
                                            'size=sm flat dense').tooltip('Set output to HIGH')
                                        ui.button(icon='unpublished', on_click=lambda pin_number=gpio_state.gpio: self.set_pin_level(pin_number, False)).props(
                                            'size=sm flat dense').tooltip('Set output to LOW')
                                        ui.space()
                                        ui.button(icon='close', on_click=context_menu.close).props(
                                            'color=black size=sm flat dense')
                                    ui.separator()
                                    with ui.grid(columns=2):
                                        ui.label().bind_text_from(gpio_state, 'level',
                                                                  lambda level: f'Level: {"High" if level else "Low"}')
                                        ui.label().bind_text_from(gpio_state, 'is_input',
                                                                  lambda is_input: f'Input: {is_input}')
                                        ui.label().bind_text_from(gpio_state, 'is_output',
                                                                  lambda is_output: f'Output: {is_output}')
                                        ui.label().bind_text_from(gpio_state, 'open_drain',
                                                                  lambda open_drain: f'OpenDrain: {open_drain}')
                                        ui.label().bind_text_from(gpio_state, 'is_pullup',
                                                                  lambda is_pullup: f'PullUp: {is_pullup}')
                                        ui.label().bind_text_from(gpio_state, 'is_pulldown',
                                                                  lambda is_pulldown: f'PullDown: {is_pulldown}')
                                        ui.label().bind_text_from(gpio_state, 'drive_strength',
                                                                  lambda drive_strength: f'Drive Strength: {drive_strength}')
                                        ui.label().bind_text_from(gpio_state, 'sleep_sel',
                                                                  lambda sleep_sel: f'Sleep Sel: {sleep_sel}')
