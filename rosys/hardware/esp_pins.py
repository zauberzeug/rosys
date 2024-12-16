from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from nicegui import ui
from nicegui.elements.mixins.value_element import ValueElement

if TYPE_CHECKING:
    from .robot_brain import RobotBrain


class StatusBulb(ValueElement):
    def __init__(self, value: bool = False) -> None:
        super().__init__(value=value, on_value_change=self.on_change, tag='span')
        self.style('height: 15px; width: 15px; margin: auto; border-radius: 50%')
        self.on_change()

    def on_change(self) -> None:
        self.style('background: radial-gradient(circle at 5px 5px, #5898D4, #4682B4);' if self.value
                   else 'background: radial-gradient(circle at 5px 5px, #D3D3D3, #A9A9A9);')


@dataclass(slots=True, kw_only=True)
class GpioPin:
    gpio: int
    level: bool = False
    is_input: bool = False
    is_output: bool = False
    open_drain: bool = False
    is_pullup: bool = False
    is_pulldown: bool = False
    drive_strength: int = 0
    sleep_sel: int = 0


class EspPins:
    """Monitor and control ESP32 GPIO pins."""

    def __init__(self, name: str, robot_brain: RobotBrain) -> None:
        self.name = name
        self.robot_brain = robot_brain
        _pin_numbers: list[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                   10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                                   21, 22, 23, 25, 26, 27,
                                   32, 33, 34, 35, 36, 37, 38, 39]
        self._pins: dict[int, GpioPin] = {pin: GpioPin(gpio=pin) for pin in _pin_numbers}

    async def update_pin(self, pin: GpioPin) -> None:
        ack = f'GPIO_Status[{pin.gpio}]|' if self.name == 'core' else f'{self.name}:'
        line = await self.robot_brain.send_and_await(f'{self.name}.get_pin_status({pin.gpio})', ack, timeout=1.0)
        if line is None:
            raise TimeoutError(f'No response from {self.name} for pin {pin.gpio}')

        # GPIO_Status[3]| Level: 1| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| DriveStrength: 2| SleepSel: 0
        # p0: GPIO_Status[0]| Level: 1| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| DriveStrength: 2| SleepSel: 0
        status_dict = {}
        for part in line.split('|')[1:]:
            key, value = part.strip().split(': ')
            status_dict[key.strip()] = int(value.strip())

        pin.level = bool(status_dict.get('Level', 0))
        pin.is_input = bool(status_dict.get('InputEn', 0))
        pin.is_output = bool(status_dict.get('OutputEn', 0))
        pin.open_drain = bool(status_dict.get('OpenDrain', 0))
        pin.is_pullup = bool(status_dict.get('Pullup', 0))
        pin.is_pulldown = bool(status_dict.get('Pulldown', 0))
        pin.drive_strength = int(status_dict.get('DriveStrength', 0))
        pin.sleep_sel = int(status_dict.get('SleepSel', 0))

    async def update_all(self) -> None:
        for pin in self._pins.values():
            await self.update_pin(pin)

    async def get_pin_level(self, pin_id: int) -> bool:
        await self.update_pin(self._pins[pin_id])
        return self._pins[pin_id].level

    async def set_pin_level(self, pin: GpioPin, level: bool) -> None:
        await self.robot_brain.send(f'{self.name}.set_pin_level({pin.gpio}, {1 if level else 0})')

    async def get_strapping_pins(self) -> list[bool]:
        """Get the state of the strapping pins.

        See https://github.com/zauberzeug/lizard/issues/75 and https://github.com/zauberzeug/lizard/pull/95.
        """
        return [await self.get_pin_level(number) for number in [0, 2, 12]]

    def developer_ui(self) -> None:
        with ui.column():
            with ui.row().classes('w-full'):
                ui.markdown(f'**ESP: {self.name}**')
                ui.space()
                ui.button(icon='refresh', on_click=self.update_all).props('flat round dense').tooltip('Update once')
            ui.separator()
            with ui.grid(columns=5):
                for pin in self._pins.values():
                    with ui.row():
                        ui.label(f'GPIO {str(pin.gpio).zfill(2)}')
                        with StatusBulb().bind_value_from(pin, 'level'):
                            self._pin_context_menu(pin)

    def _pin_context_menu(self, pin: GpioPin) -> None:
        with ui.context_menu().props('auto-close=false') as context_menu:
            with ui.card():
                with ui.row().classes('w-full'):
                    ui.label(f'GPIO {pin.gpio}')
                    ui.space()
                    ui.button(icon='refresh', on_click=lambda pin=pin: self.update_pin(pin)) \
                        .props('size=sm flat dense').tooltip('Update gpio state')
                    ui.button(icon='check_circle', on_click=lambda pin=pin: self.set_pin_level(pin, True)) \
                        .props('size=sm flat dense').tooltip('Set output to HIGH')
                    ui.button(icon='unpublished', on_click=lambda pin=pin: self.set_pin_level(pin, False)) \
                        .props('size=sm flat dense').tooltip('Set output to LOW')
                    ui.space()
                    ui.button(icon='close', on_click=context_menu.close) \
                        .props('color=black size=sm flat dense')
                ui.separator()
                with ui.grid(columns=2):
                    ui.label().bind_text_from(pin, 'level', lambda level: f'Level: {"High" if level else "Low"}')
                    ui.label().bind_text_from(pin, 'is_input', lambda is_input: f'Input: {is_input}')
                    ui.label().bind_text_from(pin, 'is_output', lambda is_output: f'Output: {is_output}')
                    ui.label().bind_text_from(pin, 'open_drain', lambda open_drain: f'OpenDrain: {open_drain}')
                    ui.label().bind_text_from(pin, 'is_pullup', lambda is_pullup: f'PullUp: {is_pullup}')
                    ui.label().bind_text_from(pin, 'is_pulldown', lambda is_pulldown: f'PullDown: {is_pulldown}')
                    ui.label().bind_text_from(pin, 'drive_strength', lambda strength: f'Drive Strength: {strength}')
                    ui.label().bind_text_from(pin, 'sleep_sel', lambda sleep_sel: f'Sleep Sel: {sleep_sel}')
