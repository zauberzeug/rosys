import abc

import numpy as np
from nicegui import Event, ui

from .. import helpers, rosys
from ..helpers import remove_indentation
from .bms_message import BmsMessage
from .bms_state import BmsState
from .expander import ExpanderHardware
from .expander_bus import ExpanderBusHardware, FieldType
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain

JBD_FIELDS: list[tuple[str, FieldType]] = [
    ('voltage', 'float'),
    ('current', 'float'),
    ('percentage', 'int'),
    ('temperature', 'float'),
    ('protections', 'int'),
    ('last_message_age', 'int'),
]
"""Properties of Lizard's ``JbdBms`` that the core mirrors from an expander bus."""
ANSWER_TOLERANCE_MS = 100
"""Milliseconds by which the dates of one BMS answer may differ, as millis and age come from different steps."""
MILLIS_RANGE = 2**32
"""The expander's millis wrap after 2**32 ms; as an int field of its frame they turn negative after 24.9 days."""


def _millis_apart(a: int, b: int) -> int:
    """Milliseconds between two dates of the expander's clock, across its wrap."""
    difference = (a - b) % MILLIS_RANGE
    return min(difference, MILLIS_RANGE - difference)


class Bms(Module, abc.ABC):
    """The BMS module communicates with a simple battery management system over a serial connection.

    The BMS module provides measured voltages as an event.
    """

    def __init__(self, *, battery_low_threshold: float | None = None, **kwargs) -> None:
        """
        :param battery_low_threshold: If provided, the BATTERY_LOW event will be emitted when the battery percentage gets below this threshold.
        """
        super().__init__(**kwargs)
        self.STATE_UPDATED = Event[BmsState]()
        """The BMS state has been updated with new data (argument: ``BmsState``)"""
        self.CHARGING_STARTED = Event[[]]()
        """The battery started charging"""
        self.CHARGING_STOPPED = Event[[]]()
        """The battery stopped charging"""
        self.BATTERY_LOW = Event[[]]()
        """The battery percentage is below the threshold"""

        self.state = BmsState()
        self.raw_data: dict = {}
        self.battery_low_threshold = battery_low_threshold

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

    def _send_charging_events(self, is_charging: bool) -> None:
        if is_charging and not self.state.is_charging:
            self.CHARGING_STARTED.emit()
        elif not is_charging and self.state.is_charging:
            self.CHARGING_STOPPED.emit()

    def _send_battery_low_event(self, percentage: float) -> None:
        if self.battery_low_threshold is None or self.state.percentage is None:
            return
        if percentage < self.battery_low_threshold and self.state.percentage >= self.battery_low_threshold:
            self.BATTERY_LOW.emit()

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
                 expander: ExpanderHardware | ExpanderBusHardware | None = None,
                 name: str = 'bms',
                 rx_pin: int = 26,
                 tx_pin: int = 27,
                 baud: int = 9600,
                 num: int = 1,
                 charge_detect_threshold: float = -0.4,
                 **kwargs,
                 ) -> None:
        """
        :param expander: If provided, the BMS module will use the expander module to create the serial connection.
            On an expander bus, Lizard's ``JbdBms`` module asks the BMS on the expander, and the core mirrors its values.
        :param name: The name of the BMS module.
        :param rx_pin: The GPIO pin number to use for the RX line.
        :param tx_pin: The GPIO pin number to use for the TX line.
        :param baud: The baud rate to use for the serial communication.
        :param num: The serial port index to use for creating the serial connection.
        :param charge_detect_threshold: The threshold current to use for charging detection.
        """
        self.name = name
        self.expander = expander
        self.expander_bus = expander if isinstance(expander, ExpanderBusHardware) else None
        self.charge_detect_threshold = charge_detect_threshold
        self._answer_millis: int | None = None
        if self.expander_bus is not None:
            # NOTE: the expander's millis date the BMS answer, so that a silent expander or BMS updates no state
            fields = [*((f'{name}.{field}', type_) for field, type_ in JBD_FIELDS), ('core.millis', 'int')]
            declarations = self.expander_bus.add_lizard_code(remove_indentation(f'''
                {name}_serial = Serial({rx_pin}, {tx_pin}, {baud}, {num})
                {name} = JbdBms({name}_serial)
            '''), fields, interval=1.0)
            # NOTE: a routine is parsed once at startup; parsing the print line with every reading stalls the core
            mirrors = ', '.join(f'{self.expander_bus.name}.{field}' for field, _ in fields)
            lizard_code = f'{declarations}\nlet {name}_read do core.print("{name}", {mirrors}); end'
        else:
            lizard_code = remove_indentation(f'''
                {name} = {expander.name + "." if expander else ""}Serial({rx_pin}, {tx_pin}, {baud}, {num})
                {name}.unmute()
            ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, **kwargs)
        rosys.on_repeat(self._request, 1.0)
        if self.expander_bus is not None:
            self.message_hooks[self.expander_bus.line_prefix] = self._handle_jbd_bms
        else:
            self.message_hooks[name] = self._handle_bms

    async def _request(self) -> None:
        if self.robot_brain.is_ready and rosys.time() > self.state.last_update + self.UPDATE_INTERVAL:
            if self.expander_bus is not None:
                if not self.expander_bus.busy:
                    await self._read_mirrors()
            else:
                await self.robot_brain.send(f'{self.name}.send(0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77)')

    async def request_cell_voltages(self) -> None:
        """Ask the BMS for its cell voltages, which arrive in ``raw_data['voltages']``."""
        if self.expander_bus is not None:
            await self.expander_bus.send_after_startup(f'{self.name}.cell_voltages()')
        else:
            await self.robot_brain.send(f'{self.name}.send(0xdd, 0xa5, 0x04, 0x00, 0xff, 0xfc, 0x77)')

    async def request_hardware_version(self) -> None:
        """Ask the BMS for its hardware version, which arrives in ``raw_data['hardware version']``."""
        if self.expander_bus is not None:
            await self.expander_bus.send_after_startup(f'{self.name}.hardware_version()')
        else:
            await self.robot_brain.send(f'{self.name}.send(0xdd, 0xa5, 0x05, 0x00, 0xff, 0xfb, 0x77)')

    def _handle_bms(self, line: str) -> None:
        skip = 2 if self.expander else 1
        words = line.split()[skip:]
        msg = BmsMessage([int(w, 16) for w in words])
        msg.check()
        result = msg.interpret()
        self.raw_data = result
        self._update_state(percentage=result.get('capacity percent'),
                           voltage=result.get('total voltage'),
                           current=result.get('current'),
                           temperature=float(np.mean(result['temperatures'])) if 'temperatures' in result else None,
                           last_update=rosys.time())

    async def _read_mirrors(self) -> None:
        line = await self.robot_brain.send_and_await(f'{self.name}_read()', f'"{self.name}"', timeout=1.0)
        if line is None:
            return
        words = line.split()[1:]
        voltage, current, temperature = float(words[0]), float(words[1]), float(words[3])
        percentage, protections, age, millis = int(words[2]), int(words[4]), int(words[5]), int(words[6])
        answer_millis = millis - age
        if _millis_apart(answer_millis, 0) < ANSWER_TOLERANCE_MS:
            return  # NOTE: no answer since the expander booted, as JbdBms counts last_message_age from the boot
        if self._answer_millis is not None and _millis_apart(answer_millis, self._answer_millis) < ANSWER_TOLERANCE_MS:
            return  # NOTE: no newer answer, or the mirrors keep their values because the expander fell silent
        self._answer_millis = answer_millis
        self.raw_data = {
            **self.raw_data,  # NOTE: keeps the cell voltages and the hardware version
            'total voltage': voltage,
            'current': current,
            'capacity percent': percentage,
            'temperature': temperature,
            'protections': protections,
            'last message age': age,
        }
        self._update_state(percentage=percentage, voltage=voltage, current=current, temperature=temperature,
                           last_update=rosys.time() - age / 1000)

    def _handle_jbd_bms(self, line: str) -> None:
        words = line.split(maxsplit=2)
        if len(words) < 3 or words[1] != self.name:
            return
        text = words[2]  # NOTE: without "p0: bms"
        if text.startswith('cell voltages:'):
            self.raw_data = {**self.raw_data, 'voltages': [float(word) for word in text.split()[2:]]}
        elif text.startswith('hardware version:'):
            self.raw_data = {**self.raw_data, 'hardware version': text.removeprefix('hardware version:').strip()}

    def _update_state(self, *, percentage: float | None, voltage: float | None, current: float | None,
                      temperature: float | None, last_update: float) -> None:
        if percentage is not None:
            self._send_battery_low_event(percentage)
        self.state.percentage = percentage
        self.state.voltage = voltage
        self.state.current = current
        self.state.temperature = temperature
        is_charging = (self.state.current or 0) > self.charge_detect_threshold
        self._send_charging_events(is_charging)
        self.state.is_charging = is_charging
        self.state.last_update = last_update
        self.STATE_UPDATED.emit(self.state)


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

    def __init__(self, *, voltage_per_second: float = 0.0, **kwargs) -> None:
        """
        :param voltage_per_second: The voltage change per second. Positive for charging, negative for discharging.
        """
        super().__init__(**kwargs)
        self.voltage_per_second = voltage_per_second
        self.state.voltage = self.AVERAGE_VOLTAGE

    async def step(self, dt: float) -> None:
        assert self.state.voltage is not None
        next_voltage = self.state.voltage + self.voltage_per_second * dt
        self.state.voltage = max(min(next_voltage, self.MAX_VOLTAGE), self.MIN_VOLTAGE)
        is_charging = self.voltage_per_second > 0
        self._send_charging_events(is_charging)
        self.state.is_charging = is_charging
        percentage = helpers.ramp(self.state.voltage, self.MIN_VOLTAGE, self.MAX_VOLTAGE, 0.0, 100.0)
        self._send_battery_low_event(percentage)
        self.state.percentage = percentage
        self.state.current = self.CHARGING_CURRENT if self.state.is_charging else self.DISCHARGING_CURRENT
        self.state.temperature = self.AVERAGE_TEMPERATURE + \
            self.TEMPERATURE_AMPLITUDE * np.sin(self.TEMPERATURE_FREQUENCY * rosys.time())
        self.state.last_update = rosys.time()
        self.STATE_UPDATED.emit(self.state)

    def developer_ui(self) -> None:
        super().developer_ui()
        ui.number(suffix='V/s', step=0.001, format='%.3f').props('dense').classes('w-20') \
            .bind_value(self, 'voltage_per_second') \
            .tooltip('Voltage change per second (positive for charging, negative for discharging)')
