import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING

from nicegui import ui
from nicegui.elements.mixins.value_element import ValueElement

from .. import rosys

if TYPE_CHECKING:
    from rosys.hardware.robot_brain import RobotBrain


class StatusBulb(ValueElement):
    # TODO: colors
    def __init__(self, value: bool = False) -> None:
        super().__init__(value=value, on_value_change=self.on_change, tag='span')
        self.style('height: 15px; width: 15px; margin: auto; border-radius: 50%')
        self.on_change()

    def on_change(self) -> None:
        self.style(add='background: radial-gradient(circle at 5px 5px, #5FE5E0, #5898D4); box-shadow: #5FE5E066 0px 0px 10px 5px;' if self.value
                   else 'background: radial-gradient(circle at 5px 5px, #2E5396, #294790);', remove='box-shadow:;')


@dataclass
class GPIOPin:
    gpio: int
    level: bool
    is_input: bool
    is_output: bool
    # TODO:
    open_drain: bool
    is_pullup: bool
    is_pulldown: bool
    # TODO:
    is_interrupt: bool
    is_strapping: bool


class ESPPins:
    """TODO

    TODO
    """

    def __init__(self, name: str, robot_brain: 'RobotBrain', update_time: float = 1.0) -> None:
        self.log = logging.getLogger('rosys.esp_pins')
        self.name = name
        self.robot_brain = robot_brain
        self.pin_numbers: list[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                       15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 37, 38, 39]
        self.gpio_states: dict[int, GPIOPin] = {pin: GPIOPin(
            pin, True, False, False, False, False, False, False, False) for pin in self.pin_numbers}
        self.auto_update: bool = False
        robot_brain.LINE_RECEIVED.register(self.parse)
        rosys.on_repeat(self.run_auto_update, update_time)

    def parse(self, line: str) -> None:
        if 'GPIO_Status' not in line:
            return
          # Split the string by the '|' character and remove any leading/trailing spaces
        parts = [part.strip() for part in line.split('|')]
        status_dict = {}

        for part in parts[1:]:
            key, value = part.split(': ')
            status_dict[key.strip()] = int(value.strip())

        gpio_str = parts[0].split('[')[1].split(']')[0]
        pin = int(gpio_str)

        self.gpio_states[pin].is_input = bool(status_dict.get('InputEn', 0))
        self.gpio_states[pin].is_output = bool(status_dict.get('OutputEn', 0))
        self.gpio_states[pin].level = bool(status_dict.get('Level'))
        self.gpio_states[pin].is_pullup = bool(status_dict.get('Pullup', 0))
        self.gpio_states[pin].is_pulldown = bool(status_dict.get('Pulldown', 0))
        self.gpio_states[pin].is_interrupt = bool(status_dict.get('is_interrupt', 0))

    async def update(self) -> None:
        for pin_number in self.pin_numbers:
            await self.update_pin(pin_number)

    async def update_pin(self, pin_number: int) -> None:
        await self.robot_brain.send(f'{self.name}.gpio_status({pin_number})')
        await rosys.sleep(0.1)

    async def run_auto_update(self) -> None:
        if not self.auto_update:
            return
        await self.update()

    def developer_ui(self):
        with ui.column():
            with ui.row().classes('w-full'):
                ui.markdown(f'**ESP: {self.name}**')
                ui.space()
                ui.button(icon='refresh', on_click=self.update).props(
                    'flat round dense').tooltip('Update once')
                ui.switch().bind_value(self, 'auto_update').tooltip('Activate auto update')
            ui.separator()
            with ui.grid(columns=5):
                for gpio_state in self.gpio_states.values():
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
                                        set_high_button = ui.button(icon='check_circle', on_click=lambda pin_number=gpio_state.gpio: self.update_pin(pin_number)).props(
                                            'size=sm flat dense').tooltip('Set output to HIGH')
                                        set_low_button = ui.button(icon='unpublished', on_click=lambda pin_number=gpio_state.gpio: self.update_pin(pin_number)).props(
                                            'size=sm flat dense').tooltip('Set output to LOW')
                                        if not gpio_state.is_output:
                                            set_high_button.disable()
                                            set_low_button.disable()
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
                                        ui.label().bind_text_from(gpio_state, 'is_interrupt',
                                                                  lambda is_interrupt: f'Interrupt: {is_interrupt}')
                                        ui.label().bind_text_from(gpio_state, 'is_strapping',
                                                                  lambda is_strapping: f'Strapping: {is_strapping}')
