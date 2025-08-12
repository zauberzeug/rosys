import abc

import numpy as np
from nicegui import ui

from .. import helpers, rosys
from ..helpers import remove_indentation
from .bms_message import BmsMessage
from .bms_state import BmsState
from .expander import ExpanderHardware
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Bms(Module, abc.ABC):
    """The BMS module communicates with a simple battery management system over a serial connection.

    The BMS module provides measured voltages as an event.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.state = BmsState()
        self.raw_data: dict = {}

    def is_above_percent(self, value: float) -> bool:
        """Returns whether the battery is charged above the given percentage."""
        return self.state.percentage is not None and self.state.percentage > value

    def is_below_percent(self, value: float) -> bool:
        """Returns whether the battery is charged below the given percentage."""
        return self.state.percentage is not None and self.state.percentage < value

    def is_above_voltage(self, value: float) -> bool:
        """Returns whether the battery voltage is above the given value."""
        return self.state.voltage is not None and self.state.voltage > value

    def is_below_voltage(self, value: float) -> bool:
        """Returns whether the battery voltage is below the given value."""
        return self.state.voltage is not None and self.state.voltage < value

    def developer_ui(self) -> None:
        ui.label('Battery Management System').classes('text-center text-bold')
        with ui.grid(columns=2).classes('gap-y-1 [&>*:nth-child(even)]:justify-self-end'):
            ui.label('Percentage:')
            ui.label().bind_text_from(self.state, 'percentage', lambda x: f'{x:.1f}%' if x is not None else 'N/A')
            ui.label('Voltage:')
            ui.label().bind_text_from(self.state, 'voltage', lambda x: f'{x:.1f}V' if x is not None else 'N/A')
            ui.label('Current:')
            ui.label().bind_text_from(self.state, 'current', lambda x: f'{x:.1f}A' if x is not None else 'N/A')
            ui.label('Temperature:')
            ui.label().bind_text_from(self.state, 'temperature', lambda x: f'{x:.1f}°C' if x is not None else 'N/A')
            ui.label('Charging:')
            ui.label().bind_text_from(self.state, 'is_charging', lambda x: 'Yes' if x else 'No')


class BmsHardware(Bms, ModuleHardware):
    """This module implements the hardware interface for the BMS module."""

    UPDATE_INTERVAL = 5.0

    def __init__(self, robot_brain: RobotBrain, *,
                 expander: ExpanderHardware | None = None,
                 name: str = 'bms',
                 rx_pin: int = 26,
                 tx_pin: int = 27,
                 baud: int = 9600,
                 num: int = 1,
                 charge_detect_threshold: float = -0.4,
                 ) -> None:
        self.name = name
        self.expander = expander
        self.charge_detect_threshold = charge_detect_threshold
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if expander else ""}Serial({rx_pin}, {tx_pin}, {baud}, {num})
            {name}.unmute()
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
        rosys.on_repeat(self._request, 1.0)
        self.message_hooks[name] = self._handle_bms

    async def _request(self) -> None:
        if self.robot_brain.is_ready and rosys.time() > self.state.last_update + self.UPDATE_INTERVAL:
            await self.robot_brain.send(f'{self.name}.send(0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77)')

    def _handle_bms(self, line: str) -> None:
        skip = 2 if self.expander else 1
        words = line.split()[skip:]
        msg = BmsMessage([int(w, 16) for w in words])
        msg.check()
        result = msg.interpret()
        self.state.percentage = result.get('capacity percent')
        self.state.voltage = result.get('total voltage')
        self.state.current = result.get('current')
        self.state.temperature = np.mean(result['temperatures']) if 'temperatures' in result else None
        self.state.is_charging = (self.state.current or 0) > self.charge_detect_threshold
        self.state.last_update = rosys.time()
        self.raw_data = result


class BmsSimulation(Bms, ModuleSimulation):
    """This module simulates a BMS module."""

    AVERAGE_VOLTAGE = 25.0
    MIN_VOLTAGE = 22.5
    MAX_VOLTAGE = 27.5
    CHARGING_CURRENT = 1.0
    DISCHARGING_CURRENT = -0.7
    AVERAGE_TEMPERATURE = 20.0
    TEMPERATURE_AMPLITUDE = 1.0
    TEMPERATURE_FREQUENCY = 0.01

    def __init__(self, *, voltage_per_second: float = 0.0) -> None:
        super().__init__()
        self.voltage_per_second = voltage_per_second
        self.state.voltage = self.AVERAGE_VOLTAGE

    async def step(self, dt: float) -> None:
        assert self.state.voltage is not None
        next_voltage = self.state.voltage + self.voltage_per_second * dt
        self.state.voltage = max(min(next_voltage, self.MAX_VOLTAGE), self.MIN_VOLTAGE)
        self.state.is_charging = self.voltage_per_second > 0
        self.state.percentage = helpers.ramp(self.state.voltage, self.MIN_VOLTAGE, self.MAX_VOLTAGE, 0.0, 100.0)
        self.state.current = self.CHARGING_CURRENT if self.state.is_charging else self.DISCHARGING_CURRENT
        self.state.temperature = self.AVERAGE_TEMPERATURE + \
            self.TEMPERATURE_AMPLITUDE * np.sin(self.TEMPERATURE_FREQUENCY * rosys.time())
        self.state.last_update = rosys.time()

    def developer_ui(self) -> None:
        super().developer_ui()
        ui.number(suffix='V/s', step=0.001, format='%.3f').props('dense').classes('w-20') \
            .bind_value(self, 'voltage_per_second') \
            .tooltip('Voltage change per second (positive for charging, negative for discharging)')
