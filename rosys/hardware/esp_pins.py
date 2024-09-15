import logging
from nicegui import ui
from ..hardware import RobotBrain


from nicegui.elements.mixins.value_element import ValueElement
from dataclasses import dataclass


class StatusBulb(ValueElement):
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
    interrupt: bool
    is_strapping: bool


class ESPPins:
    """TODO

    TODO
    """

    def __init__(self, name: str, robot_brain: RobotBrain) -> None:
        self.log = logging.getLogger('rosys.esp_pins')
        self.name = name
        self.robot_brain = robot_brain

        self.pin_numbers: list[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                       15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 37, 38, 39]
        self.gpio_states: dict[int, GPIOPin] = {pin: GPIOPin(
            pin, True, False, False, False, False, False, False, False) for pin in self.pin_numbers}

        robot_brain.LINE_RECEIVED.register(self.parse)

    def parse(self, line: str) -> None:
        self.log.info("received: %s", line)

    def developer_ui(self):
        with ui.card().style('min-width: 200px; background-color: #3E63A6; color: white;'):
            ui.markdown(f'**ESP: {self.name}**').classes('w-full text-center')
            ui.separator().style('background-color: white;')
            with ui.grid(columns=4):
                for gpio_state in self.gpio_states.values():
                    with ui.row():
                        ui.label(f'GPIO{gpio_state.gpio}')
                        with StatusBulb().bind_value_from(gpio_state, 'level'):
                            with ui.tooltip():
                                with ui.column():
                                    ui.label(f'GPIO {gpio_state.gpio}')
                                    ui.label().bind_text_from(gpio_state, 'level',
                                                              lambda level: f'Level: {"High" if level else "Low"}')
                                    ui.label().bind_text_from(gpio_state, 'is_input',
                                                              lambda is_input: f'Input: {is_input}')
                                    ui.label().bind_text_from(gpio_state, 'is_output',
                                                              lambda is_output: f'Output: {is_output}')
                                    ui.label().bind_text_from(gpio_state, 'is_pullup',
                                                              lambda is_pullup: f'PullUp: {is_pullup}')
