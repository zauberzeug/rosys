import abc
import dataclasses
import logging
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass, field

from nicegui import Event, ui

from .. import rosys
from .bms import Bms, BmsSimulation
from .can import CanHardware
from .module import ModuleHardware
from .robot_brain import EspNotReadyException, RobotBrain

PREFIX = 0x1F
SHUTDOWN = 0x00
ERROR = 0x01
MEASUREMENT_PHASE = 0x08
HEARTBEAT = 0x10
BMS_1 = 0x11
BMS_2 = 0x12
BMS_FIXED = 0x13
IDENTITY = 0x18
DIAGNOSTICS = 0x19
CELL_REQUEST = 0x20
CELLS = 0x21

SHUTDOWN_MAGIC = b'AKKU-AUS'
UNKNOWN_U8 = 0xFF
UNKNOWN_U16 = 0xFFFF

ERRORS = {
    0x01: ('ESP missing', logging.WARNING),
    0x02: ('BMS does not answer, relay off', logging.ERROR),
    0x03: ('BMS reports a protection flag, relay off', logging.ERROR),
    0x04: ('duplicate node ID, relay off', logging.ERROR),
    0x05: ('CAN controller restarted after bus-off', logging.WARNING),
    0x06: ('reset reason does not match RTC memory, relay off', logging.ERROR),
    0x07: ('announcement unanswered', logging.WARNING),
    0x08: ('error cleared', logging.INFO),
    0x09: ('restart loop, relay released', logging.ERROR),
    0x0A: ('battery bus not connected', logging.WARNING),
}


@dataclass(slots=True, kw_only=True)
class BatteryPack:
    """State of one battery pack as reported by its relay ESP over CAN."""
    node_id: int
    last_heartbeat: float = 0.0
    protocol_version: int | None = None
    relay_on: bool = False
    bus_voltage: bool = False
    bms_valid: bool = False
    load_free_valid: bool = False
    error_active: bool = False
    shut_down: bool = False
    measuring: bool = False
    load_free_voltage: float | None = None
    voltage: float | None = None
    current: float | None = None
    remaining_capacity: float | None = None
    nominal_capacity: float | None = None
    protection_flags: int | None = None
    percentage: float | None = None
    fet_state: int | None = None
    temperatures: list[float] = field(default_factory=list)
    cycles: int | None = None
    production_date: str | None = None
    bms_version: int | None = None
    cell_count: int | None = None
    sensor_count: int | None = None
    cell_voltages: list[float | None] = field(default_factory=list)
    balancing: int | None = None
    chip_id: str | None = None
    firmware_version: str | None = None
    reset_reason: int | None = None
    bus_off_count: int | None = None
    bms_timeout_count: int | None = None
    restart_count: int | None = None
    diagnostic_flags: int | None = None
    last_message: str | None = None

    @property
    def name(self) -> str:
        return self.chip_id or f'{self.node_id:04x}'

    @property
    def is_usable(self) -> bool:
        """Whether the pack has valid BMS data and is not off because of an error or the shutdown latch."""
        return self.bms_valid and not self.error_active and not self.shut_down


class BmsCan(Bms, abc.ABC):
    """A battery of several packs, each switched by an ESP that reports it over CAN.

    Besides the ``Bms`` interface it can start a load-free measurement phase, request cell voltages and switch off all relays.
    """

    MEASUREMENT_PHASE_DURATION = 4.0
    """Time the load has to stay low after a measurement phase was started (s)"""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.MEASUREMENT_PHASE_STARTED = Event[[]]()
        """A load-free measurement phase was started; keep the load low for ``MEASUREMENT_PHASE_DURATION`` seconds"""
        self.packs: dict[int, BatteryPack] = {}
        self.last_measurement_phase = -float('inf')
        self.is_shutting_down = False

    @abc.abstractmethod
    async def start_measurement_phase(self) -> None:
        """Let every ESP measure its load-free voltage for 3 s, so that a pack that is off can join.

        The load has to stay low for ``MEASUREMENT_PHASE_DURATION`` seconds.
        """

    @abc.abstractmethod
    async def request_cell_voltages(self, node_id: int) -> None:
        """Ask the ESP of one pack for its cell voltages and balancing state."""

    @abc.abstractmethod
    async def shut_down(self) -> None:
        """Switch off the relays of all packs until the power switch is turned off and on again."""


class BmsCanHardware(BmsCan, ModuleHardware):
    """This module reads several battery packs over CAN, each reported by the ESP that switches its relay.

    The ESPs decide on their own which pack is connected to the battery bus.
    This module only listens to their extended CAN frames, combines the packs into one ``BmsState``
    and sends the few commands the ESPs accept from the robot brain:
    start a load-free measurement phase, request cell voltages and switch off all relays.
    It needs a Lizard version whose CAN module provides ``output_extended``.
    """

    PACK_TIMEOUT = 3.5
    """Time after the last heartbeat until a pack counts as missing (s)"""
    SHUTDOWN_REPEAT_INTERVAL = 0.1
    SHUTDOWN_CHECK_DELAY = 2.0

    def __init__(self, robot_brain: RobotBrain, *,
                 can: CanHardware,
                 charge_detect_threshold: float = -0.4,
                 measurement_phase_interval: float = 60.0,
                 is_load_low: Callable[[], bool] | None = None,
                 **kwargs) -> None:
        """
        :param can: The CAN module the battery ESPs are connected to.
        :param charge_detect_threshold: The summed current above which the battery counts as charging (A).
        :param measurement_phase_interval: Minimum time between two automatic measurement phases (s).
        :param is_load_low: If provided, a measurement phase is started automatically
            while a pack is off without an error and this function returns ``True``.
            The caller has to keep the load low for ``MEASUREMENT_PHASE_DURATION`` seconds after ``MEASUREMENT_PHASE_STARTED``.
        """
        self.can = can
        self.charge_detect_threshold = charge_detect_threshold
        self.measurement_phase_interval = measurement_phase_interval
        self.is_load_low = is_load_low
        mask = PREFIX << 24
        lizard_code = f'{can.name}.output_extended({mask:#010x}, {mask:#010x})'
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, **kwargs)
        self._frame_handlers: dict[int, tuple[int, Callable[[BatteryPack, bytes], None]]] = {
            ERROR: (7, self._handle_error),
            HEARTBEAT: (8, self._handle_heartbeat),
            BMS_1: (8, self._handle_bms_1),
            BMS_2: (8, self._handle_bms_2),
            BMS_FIXED: (7, self._handle_bms_fixed),
            IDENTITY: (8, self._handle_identity),
            DIAGNOSTICS: (8, self._handle_diagnostics),
            CELLS: (7, self._handle_cells),
        }
        """Minimum data length and handler per message type"""
        self.message_hooks[can.name] = self._handle_can
        rosys.on_repeat(self._update_state, 1.0)

    def is_fresh(self, pack: BatteryPack) -> bool:
        """Whether the pack's heartbeat is not older than ``PACK_TIMEOUT``."""
        return rosys.time() - pack.last_heartbeat < self.PACK_TIMEOUT

    @property
    def fresh_packs(self) -> list[BatteryPack]:
        """Packs whose heartbeat is not older than ``PACK_TIMEOUT``."""
        return [pack for pack in self.packs.values() if self.is_fresh(pack)]

    def is_outdated(self, pack: BatteryPack) -> bool:
        """Whether the pack speaks an older protocol version than another pack on the bus."""
        versions = [p.protocol_version for p in self.fresh_packs if p.protocol_version is not None]
        return pack.protocol_version is not None and pack.protocol_version < max(versions, default=0)

    async def start_measurement_phase(self) -> None:
        self.log.info('starting a load-free measurement phase')
        self.last_measurement_phase = rosys.time()
        await self._send(MEASUREMENT_PHASE, 0)
        self.MEASUREMENT_PHASE_STARTED.emit()

    async def request_cell_voltages(self, node_id: int) -> None:
        await self._send(CELL_REQUEST, node_id)

    async def shut_down(self) -> None:
        """Switch off the relays of all packs until the power switch is turned off and on again.

        The command is repeated every 100 ms until the robot brain loses power.
        """
        if self.is_shutting_down:
            return
        self.is_shutting_down = True
        self.log.warning('switching off all battery relays')
        start = rosys.time()
        reported = False
        try:
            while True:
                with suppress(EspNotReadyException):  # NOTE: keep repeating until the Core is ready again
                    await self._send(SHUTDOWN, 0, SHUTDOWN_MAGIC)
                if not reported and rosys.time() > start + self.SHUTDOWN_CHECK_DELAY:
                    reported = True
                    if remaining := [pack.name if self.is_fresh(pack) else f'{pack.name} (no heartbeat)'
                                     for pack in self.packs.values() if pack.relay_on]:
                        rosys.notify(f'Battery relays still on: {", ".join(remaining)}', 'negative')
                await rosys.sleep(self.SHUTDOWN_REPEAT_INTERVAL)
        finally:
            self.is_shutting_down = False

    async def _send(self, message_type: int, node_id: int, data: bytes = bytes(8)) -> None:
        identifier = PREFIX << 24 | message_type << 16 | node_id
        await self.robot_brain.send(f'{self.can.name}.send({identifier:#010x}, {", ".join(str(b) for b in data)})')

    def _handle_can(self, line: str) -> None:
        identifier, *values = line.rsplit(maxsplit=1)[-1].split(',')
        if len(identifier) != 8:
            return
        frame_id = int(identifier, 16)
        if frame_id >> 24 != PREFIX:
            return
        message_type = frame_id >> 16 & 0xFF
        node_id = frame_id & 0xFFFF
        data = bytes(int(value, 16) for value in values)
        if message_type not in self._frame_handlers:
            return
        min_length, handler = self._frame_handlers[message_type]
        if len(data) < min_length:
            self.log.warning('battery frame %s is too short', identifier)
            return
        pack = self.packs.setdefault(node_id, BatteryPack(node_id=node_id))
        handler(pack, data)

    def _handle_heartbeat(self, pack: BatteryPack, data: bytes) -> None:
        flags = data[0]
        pack.relay_on = bool(flags & 1 << 0)
        pack.bus_voltage = bool(flags & 1 << 1)
        pack.bms_valid = bool(flags & 1 << 2)
        pack.load_free_valid = bool(flags & 1 << 3)
        pack.error_active = bool(flags & 1 << 4)
        pack.shut_down = bool(flags & 1 << 5)
        pack.measuring = bool(flags & 1 << 6)
        pack.protocol_version = data[1] >> 4
        pack.load_free_voltage = _scaled(_u16(data, 2), 0.01) if pack.load_free_valid else None
        pack.voltage = _scaled(_u16(data, 4), 0.01) if pack.bms_valid else None
        pack.current = int.from_bytes(data[6:8], 'little', signed=True) * 0.01 if pack.bms_valid else None
        pack.last_heartbeat = rosys.time()

    def _handle_bms_1(self, pack: BatteryPack, data: bytes) -> None:
        pack.remaining_capacity = _scaled(_u16(data, 0), 0.01)
        pack.nominal_capacity = _scaled(_u16(data, 2), 0.01)
        pack.protection_flags = _unknown_to_none(_u16(data, 4), UNKNOWN_U16)
        pack.percentage = _unknown_to_none(data[6], UNKNOWN_U8)
        pack.fet_state = _unknown_to_none(data[7], UNKNOWN_U8)

    def _handle_bms_2(self, pack: BatteryPack, data: bytes) -> None:
        temperatures = [_u16(data, i) for i in range(0, 8, 2)]
        pack.temperatures = [round(t * 0.1 - 273.15, 1) for t in temperatures if t != UNKNOWN_U16]

    def _handle_bms_fixed(self, pack: BatteryPack, data: bytes) -> None:
        pack.cycles = _unknown_to_none(_u16(data, 0), UNKNOWN_U16)
        date = _u16(data, 2)
        pack.production_date = None if date == UNKNOWN_U16 else \
            f'{2000 + (date >> 9)}-{date >> 5 & 0x0F:02d}-{date & 0x1F:02d}'
        pack.bms_version = _unknown_to_none(data[4], UNKNOWN_U8)
        pack.cell_count = _unknown_to_none(data[5], UNKNOWN_U8)
        pack.sensor_count = _unknown_to_none(data[6], UNKNOWN_U8)

    def _handle_identity(self, pack: BatteryPack, data: bytes) -> None:
        pack.chip_id = ':'.join(f'{b:02x}' for b in reversed(data[0:6]))
        version = _u16(data, 6)
        pack.firmware_version = f'{version >> 8}.{version & 0xFF}'

    def _handle_diagnostics(self, pack: BatteryPack, data: bytes) -> None:
        pack.reset_reason = data[0]
        pack.bus_off_count = _u16(data, 1)
        pack.bms_timeout_count = _u16(data, 3)
        pack.restart_count = _u16(data, 5)
        pack.diagnostic_flags = data[7]

    def _handle_cells(self, pack: BatteryPack, data: bytes) -> None:
        part = data[0]
        if part == 0:
            pack.balancing = int.from_bytes(data[2:6], 'little')
            pack.cell_voltages = [None] * data[1]
            return
        for i in range(3):
            cell = 3 * (part - 1) + i
            if cell < len(pack.cell_voltages):
                pack.cell_voltages[cell] = _scaled(_u16(data, 1 + 2 * i), 0.001)

    def _handle_error(self, pack: BatteryPack, data: bytes) -> None:
        code = data[0]
        affected = _u16(data, 1)
        detail = int.from_bytes(data[3:7], 'little')
        text, level = ERRORS.get(code, (f'unknown code {code:#04x}', logging.ERROR))
        message = f'battery {pack.name}: {text} (code {code:#04x}, node {affected:04x}, detail {detail})'
        pack.last_message = message
        self.log.log(level, message)
        if level >= logging.ERROR:
            rosys.notify(message, 'negative')

    async def _update_state(self) -> None:
        packs = [pack for pack in self.fresh_packs if pack.bms_valid]
        usable = [p for p in packs if p.is_usable]
        remaining = [p.remaining_capacity for p in usable if p.remaining_capacity is not None]
        nominal = [p.nominal_capacity for p in usable if p.nominal_capacity is not None]
        connected = [p.voltage for p in packs if p.relay_on and p.voltage is not None]
        temperatures = [t for p in packs for t in p.temperatures]
        currents = [p.current for p in packs if p.current is not None]
        is_complete = len(remaining) == len(nominal) == len(usable) and sum(nominal) > 0
        percentage = 100 * sum(remaining) / sum(nominal) if is_complete else None
        if percentage is not None:
            self._send_battery_low_event(percentage)
        self.state.percentage = percentage
        self.state.voltage = sum(connected) / len(connected) if connected else None
        self.state.current = sum(currents) if currents else None
        self.state.temperature = max(temperatures) if temperatures else None
        is_charging = self.state.current is not None and self.state.current > self.charge_detect_threshold
        self._send_charging_events(is_charging)
        self.state.is_charging = is_charging
        if packs:
            self.state.last_update = rosys.time()
        self.raw_data = {pack.name: dataclasses.asdict(pack) for pack in self.packs.values()}
        self.STATE_UPDATED.emit(self.state)
        await self._start_measurement_phase_if_needed()

    async def _start_measurement_phase_if_needed(self) -> None:
        if self.is_load_low is None or self.is_shutting_down or not self.robot_brain.is_ready:
            return
        if rosys.time() < self.last_measurement_phase + self.measurement_phase_interval:
            return
        if not any(not p.relay_on and p.is_usable and not self.is_outdated(p) for p in self.fresh_packs):
            return
        if not self.is_load_low():
            return
        await self.start_measurement_phase()

    def developer_ui(self) -> None:
        super().developer_ui()
        label = ui.label().classes('whitespace-pre-line')
        ui.timer(1.0, lambda: label.set_text('\n'.join(self._pack_summary(p) for p in self.packs.values())))

    def _pack_summary(self, pack: BatteryPack) -> str:
        if not self.is_fresh(pack):
            return f'{pack.name}: missing'
        parts = [
            'on' if pack.relay_on else 'off',
            f'{pack.voltage:.2f}V' if pack.voltage is not None else '',
            f'{pack.current:.2f}A' if pack.current is not None else '',
            f'{pack.percentage}%' if pack.percentage is not None else '',
            'error' if pack.error_active else '',
            'outdated' if self.is_outdated(pack) else '',
        ]
        return f'{pack.name}: {", ".join(part for part in parts if part)}'


class BmsCanSimulation(BmsCan, BmsSimulation):
    """This module simulates the battery as one pack; it has no ``packs`` and sends no commands."""

    async def start_measurement_phase(self) -> None:
        self.last_measurement_phase = rosys.time()
        self.MEASUREMENT_PHASE_STARTED.emit()

    async def request_cell_voltages(self, node_id: int) -> None:
        pass

    async def shut_down(self) -> None:
        self.log.warning('switching off all battery relays')
        self.is_shutting_down = True


def _u16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], 'little')


def _unknown_to_none(value: int, unknown: int) -> int | None:
    return None if value == unknown else value


def _scaled(value: int, factor: float) -> float | None:
    return None if value == UNKNOWN_U16 else round(value * factor, 3)
