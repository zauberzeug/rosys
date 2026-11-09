import asyncio
import logging
from collections import defaultdict
from collections.abc import Sequence
from typing import Literal

from nicegui import background_tasks, ui

from .. import rosys
from .module import ModuleHardware
from .robot_brain import RobotBrain
from .serial import SerialHardware

FieldType = Literal['bool', 'int', 'float']

CORE_ID = 1
"""The core's node on the bus."""
PEER_ID = 2
"""The expander's node on the bus."""
CONSOLE_RX_PIN = 3
CONSOLE_TX_PIN = 1
"""The console pins of the expander (an ESP32), which its bus link takes over."""
PAYLOAD_LIMIT = 255
"""Bytes of a message to the expander; Lizard's ``SerialBus`` refuses 256 and more."""
FRAME_PAYLOAD = 180
"""Bytes of values in one telemetry frame (Lizard's ``telemetry::MAX_PAYLOAD``)."""
MAX_FRAMES = 8
"""Telemetry frames a bus peer keeps slots for."""
FLASH_TIMEOUT = 180.0
"""Seconds ``flash()`` may take; the core's main loop stalls meanwhile."""
READY_TIMEOUT = 10.0
"""Seconds until a restarted expander answers the core's poll with ``Ready.``."""
STARTUP_TIMEOUT = 10.0
"""Seconds a command waits for the expander to run its startup script."""
LINE_INTERVAL = 0.05
"""Seconds between the startup lines that ``configure()`` writes, so that they do not fill the bus's queue to the expander."""


class ExpanderBusHardware(ModuleHardware):
    """A port expander as the only peer of a Lizard ``SerialBus`` with the core, on the expander's console pins.

    It replaces ``ExpanderHardware`` for an expander that is wired the same way.
    The expander runs a startup script of its own, which modules extend with ``add_lizard_code``,
    and sends their values in telemetry frames, which a ``BusTelemetry`` module of the core mirrors as its properties,
    e.g. ``p0_values.button.level``.
    Lines of the expander arrive as ``p0[2]: ...``, with the name of the bus and the expander's node;
    commands go to the expander with ``send``.

    ``configure()`` writes the expander's startup script over the bus;
    ``flash()`` copies the core's firmware onto the expander through its boot and enable pins with ``flash_peer``
    and configures it afterwards, since flashing erases it.
    The core keeps both pins high with two outputs, since the expander runs only while its EN is driven high.
    """

    def __init__(self, robot_brain: RobotBrain, *,
                 name: str = 'p0',
                 serial: SerialHardware,
                 boot: int = 25,
                 enable: int = 14) -> None:
        """
        :param name: name of the core's ``SerialBus`` to the expander
        :param serial: the core's serial to the expander's console
        :param boot: the core's GPIO wired to the expander's GPIO0, an output ``<name>_boot`` that stays on
        :param enable: the core's GPIO wired to the expander's EN, an output ``<name>_enable`` that stays on
        """
        self.name = name
        self.serial = serial
        self.boot = boot
        self.enable = enable
        lizard_code = '\n'.join([
            f'{self.boot_name} = Output({boot})',
            f'{self.boot_name}.on()',  # NOTE: before EN, so that the expander starts its firmware, not its bootloader
            f'{self.enable_name} = Output({enable})',
            f'{self.enable_name}.on()',
            f'{name} = SerialBus({serial.name}, {CORE_ID})',
            f'{name}.make_coordinator({PEER_ID})',
            f'{self.mirror_name} = BusTelemetry({name}, {PEER_ID})',
        ])
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
        self.log = logging.getLogger('rosys.expander_bus')
        self.busy = False
        """Whether the expander is being flashed or configured, so that it may be silent or restart."""
        self.runs_startup = False
        """Whether the expander reported the checksum of ``startup`` and no startup error since it last restarted."""
        self._startup_failed = False
        self._code: list[str] = []
        self._frames: dict[int, dict[str, FieldType]] = {}
        self._replies: dict[str, str | None] = {}
        self._reply_locks: defaultdict[str, asyncio.Lock] = defaultdict(asyncio.Lock)
        self._ready_count = 0
        self._flashing = False
        self._flash_result: str | None = None
        robot_brain.LINE_RECEIVED.subscribe(self._handle_line)
        robot_brain.ESP_CONNECTED.subscribe(self._check_after_connect)
        robot_brain.lizard_firmware.expander_bus = self
        robot_brain.esp_pins_p0.expander_bus = self

    @property
    def line_prefix(self) -> str:
        """The core prints the expander's lines as ``p0[2]: <line>``, unlike ``bus[2]:`` of a peer 2 of another bus."""
        return f'{self.name}[{PEER_ID}]:'

    @property
    def boot_name(self) -> str:
        """Name of the core's output to the expander's GPIO0."""
        return f'{self.name}_boot'

    @property
    def enable_name(self) -> str:
        """Name of the core's output to the expander's EN."""
        return f'{self.name}_enable'

    @property
    def mirror_name(self) -> str:
        """Name of the core's ``BusTelemetry`` module that mirrors the expander's frames."""
        return f'{self.name}_values'

    def mirror(self, field: str) -> str:
        """The core's property that mirrors ``field`` of the expander."""
        return f'{self.mirror_name}.{field}'

    def add_lizard_code(self, code: str, fields: Sequence[tuple[str, FieldType]] = (), *,
                        interval: float = 0.1) -> str:
        """Add ``code`` to the expander's startup script and send ``fields`` to the core every ``interval`` seconds.

        :param code: Lizard code for the expander, without the ``p0.`` of an expander's proxies
        :param fields: properties or variables of the expander with their Lizard types
        :param interval: seconds between the expander's frames with these fields
        :return: the core's declarations of the mirrored fields, which the core reads as ``mirror(field)``
        :raises ValueError: if a line exceeds a bus message, a field is mirrored already,
            or the fields exceed the expander's telemetry frames
        """
        lines = code.strip('\n').splitlines()
        for line in lines:
            if len(f'!+{line}'.encode()) > PAYLOAD_LIMIT:
                raise ValueError(f'a startup line of {self.name} exceeds a bus message: {line[:60]}...')
        frames = {interval_ms: dict(frame) for interval_ms, frame in self._frames.items()}
        for field, type_ in fields:
            if any(field in frame for frame in frames.values()):
                raise ValueError(f'{field} of {self.name} is mirrored already')
            frames.setdefault(round(interval * 1000), {})[field] = type_
        if len(frames) > MAX_FRAMES:
            raise ValueError(f'{self.name} sends at most {MAX_FRAMES} telemetry frames, one per interval')
        for interval_ms, frame in frames.items():
            bools = sum(type_ == 'bool' for type_ in frame.values())
            if 4 * (len(frame) - bools) + (bools + 7) // 8 > FRAME_PAYLOAD:
                raise ValueError(f'the fields of {self.name} every {interval_ms} ms exceed one telemetry frame')
            if len(f'!+{_telemetry_line(frame, interval_ms)}'.encode()) > PAYLOAD_LIMIT:
                raise ValueError(f'the telemetry line of {self.name} every {interval_ms} ms exceeds a bus message')
        self._code += lines
        self._frames = frames
        return '\n'.join(f'{type_} {self.mirror(field)} = {"false" if type_ == "bool" else "0"}'
                         for field, type_ in fields)

    @property
    def startup(self) -> str:
        """The expander's startup script, its console as bus link first, so that errors of later lines reach the core.

        ``flash()`` writes these first two lines as well.
        """
        lines = [
            f'serial = Serial({CONSOLE_RX_PIN}, {CONSOLE_TX_PIN}, {self.serial.baud}, 0)',
            f'bus = SerialBus(serial, {PEER_ID})',
            *self._code,
            *(_telemetry_line(frame, interval_ms) for interval_ms, frame in sorted(self._frames.items())),
        ]
        return ''.join(f'{line}\n' for line in lines)

    @property
    def local_checksum(self) -> str:
        """Checksum of ``startup`` the way the expander's ``core.startup_checksum()`` computes it."""
        return f'{sum(self.startup.encode()) % 0x10000:04x}'

    async def send(self, command: str, *, force: bool = False) -> None:
        """Run ``command`` on the expander, with ``<name>.send(2, "<command>")`` on the core.

        :raises ValueError: if the command exceeds a bus message
        :raises EspNotReadyException: if the core is not ready and ``force`` is ``False``
        """
        if len(command.encode()) > PAYLOAD_LIMIT:
            raise ValueError(f'a command for {self.name} exceeds a bus message: {command[:60]}...')
        escaped = command.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')  # a printf format in a string
        await self.robot_brain.send(f'{self.name}.send({PEER_ID}, "{escaped}")', force=force)

    async def send_after_startup(self, command: str, *, timeout: float = STARTUP_TIMEOUT) -> bool:
        """Run ``command`` on the expander once it runs its startup script, which creates what the command uses.

        :return: whether the command was sent; if the expander does not run its startup script in time, it notifies
        """
        deadline = rosys.time() + timeout
        while not self.runs_startup and rosys.time() < deadline:
            if not self.busy:
                await self.check_startup()
            if not self.runs_startup:
                await rosys.sleep(1.0)
        if not self.runs_startup:
            rosys.notify(f'"{command}" was not sent, since {self.name} does not run its startup script', 'warning',
                         log_level=logging.WARNING)
            return False
        await self.send(command)
        return True

    async def request(self, command: str, reply: str, *, timeout: float = 3.0, force: bool = False) -> str | None:
        """Run ``command`` on the expander and await its first line that starts with ``reply``.

        Requests with the same ``reply`` are serialized, since replies carry no request ID.

        :return: the expander's line without ``p0[2]:``, or ``None`` after the timeout
        """
        async with self._reply_locks[reply]:
            self._replies[reply] = None
            try:
                await self.send(command, force=force)
                deadline = rosys.time() + timeout
                while self._replies[reply] is None and rosys.time() < deadline:
                    await rosys.sleep(0.1)
                return self._replies[reply]
            finally:
                del self._replies[reply]

    async def read_checksum(self, *, timeout: float = 3.0) -> str | None:
        """The checksum of the expander's startup script, or ``None`` if it does not answer in time."""
        line = await self.request('core.startup_checksum()', 'checksum:', timeout=timeout, force=True)
        return line.split()[-1] if line else None

    async def check_startup(self) -> str | None:
        """Compare the expander's startup script with ``startup`` and update ``runs_startup``.

        :return: why the expander does not run ``startup``, or ``None`` if it does
        """
        checksum = await self.read_checksum()
        if not self.busy:  # NOTE: configure() and flash() decide while they restart the expander
            self.runs_startup = checksum == self.local_checksum and not self._startup_failed
        if checksum is None:
            return f'{self.name} does not answer on its serial bus'
        if checksum != self.local_checksum:
            return f'{self.name} runs startup {checksum} instead of {self.local_checksum}; please configure {self.name}'
        if self._startup_failed:
            return f'the startup script of {self.name} failed; please check its errors'
        return None

    async def configure(self) -> bool:
        """Replace the expander's startup script with ``startup`` and restart the expander.

        Like the core's, the script is verified before it is saved and once more after the restart.

        :return: whether the expander runs ``startup``
        """
        if not self.robot_brain.is_ready:
            rosys.notify(f'Configuring {self.name} failed: Robot Brain is not ready', 'negative')
            return False
        if self.busy:
            rosys.notify(f'{self.name} is being flashed or configured already', 'warning')
            return False
        rosys.notify(f'Configuring {self.name}...')
        self.busy = True
        self.runs_startup = False
        try:
            problem = await self._write_startup()
            self.runs_startup = problem is None
        finally:
            self.busy = False
        if problem is not None:
            rosys.notify(f'Configuring {self.name} failed: {problem}', 'negative', log_level=logging.ERROR)
            return False
        rosys.notify(f'{self.name} configured', 'positive')
        return True

    async def _write_startup(self) -> str | None:
        """Write, save and verify ``startup``, restarting the expander.

        :return: why the expander does not run ``startup``, or ``None`` if it does
        """
        await self.send('!-')
        for line in self.startup.splitlines():
            await self.send(f'!+{line}')
            await rosys.sleep(LINE_INTERVAL)
        checksum = await self.read_checksum() or await self.read_checksum()  # one lost answer must not fail
        if checksum != self.local_checksum:
            await self._restart()  # NOTE: reloads the saved script, which the upload replaced in RAM only
            return f'checksum {checksum} instead of {self.local_checksum}; nothing was saved'
        await self.send('!.')
        await self.read_checksum()  # NOTE: the expander answers in order, i.e. after it saved
        if not await self._restart():
            return 'no "Ready." after its restart'
        checksum = await self.read_checksum()
        if checksum != self.local_checksum:
            return f'checksum {checksum} instead of {self.local_checksum} after its restart'
        if self._startup_failed:
            return 'its startup script failed'
        return None

    async def flash(self, *, timeout: float = FLASH_TIMEOUT) -> bool:
        """Copy the core's firmware onto the expander with ``flash_peer``, then write the expander's startup script.

        The core's main loop stalls for the whole transfer (about 50 s), so the robot must be at rest.

        :return: whether the expander runs the core's firmware and ``startup``
        """
        if self.busy:
            rosys.notify(f'{self.name} is being flashed or configured already', 'warning')
            return False
        firmware = self.robot_brain.lizard_firmware
        firmware.read_local_checksum()
        await firmware.read_core_checksum()
        if not firmware.checksums_match:
            rosys.notify(f'Flashing {self.name} failed: configure the core with its expander bus first', 'negative')
            return False
        rosys.notify(f'Flashing {self.name}, the core pauses for about a minute...')
        self.busy = True
        self.runs_startup = False
        try:
            ready_count = self._ready_count
            self._flash_result = None
            self._flashing = True
            try:
                await self.robot_brain.send(f'{self.name}.flash_peer({PEER_ID}, {self.boot_name}, {self.enable_name})')
                deadline = rosys.time() + timeout
                while self._flash_result is None and rosys.time() < deadline:
                    await rosys.sleep(0.1)
            finally:
                self._flashing = False
            if self._flash_result != 'flashed':
                rosys.notify(f'Flashing {self.name} failed: {self._flash_result or "the core did not finish in time"}',
                             'negative', log_level=logging.ERROR)
                return False
            if not await self._wait_for_ready(ready_count):
                rosys.notify(f'{self.name} was flashed but does not report "Ready."', 'negative',
                             log_level=logging.ERROR)
                return False
        finally:
            self.busy = False
        return await self.configure()

    async def _restart(self) -> bool:
        ready_count = self._ready_count
        await self.send('core.restart()')
        return await self._wait_for_ready(ready_count)

    async def _wait_for_ready(self, ready_count: int) -> bool:
        deadline = rosys.time() + READY_TIMEOUT
        while self._ready_count == ready_count and rosys.time() < deadline:
            await rosys.sleep(0.1)
        return self._ready_count != ready_count

    def _handle_line(self, line: str) -> None:
        if self._flashing:
            self._handle_flash_line(line)
        # e.g. a mirrored field that no frame of the expander carries
        if line.startswith(f'warning: {self.mirror_name}:'):
            if self.busy:
                self.log.warning(line)  # NOTE: a freshly flashed expander sends no frames until it is configured
            else:
                rosys.notify(line, 'warning', log_level=logging.WARNING)
            return
        if line.startswith((f'warning: serial bus {self.name} ', f'serial bus {self.name}:')):
            self.log.warning(line)  # NOTE: e.g. a poll timeout and the answer after it whenever the expander restarts
            return
        if not line.startswith(self.line_prefix):
            return
        payload = line[len(self.line_prefix):].strip()
        for reply, answer in self._replies.items():
            if answer is None and payload.startswith(reply):
                self._replies[reply] = payload
                return
        if payload == 'Ready.':
            self._ready_count += 1
            self.runs_startup = False
            self._startup_failed = False
            if not self.busy:
                self.log.warning('%s restarted', self.name)
                background_tasks.create(self._check_after_restart(), name=f'{self.name}_check_after_restart')
        elif 'startup' in payload or payload.startswith('restoring serial bus from backup'):
            # NOTE: e.g. the boot guard skipped the startup script, so that its modules are missing
            self.runs_startup = False
            self._startup_failed = True
            rosys.notify(f'{self.name}: {payload}', 'negative', log_level=logging.ERROR)
        elif payload.startswith(('error', 'warning')):
            self.log.warning('%s: %s', self.name, payload)

    def _handle_flash_line(self, line: str) -> None:
        if line == f'serial bus {self.name} flashed peer {PEER_ID}, which restarts now':
            self._flash_result = 'flashed'
        elif line.startswith('error processing uart0:') and \
                any(f'"{name}"' in line for name in (self.name, self.serial.name, self.boot_name, self.enable_name)):
            self._flash_result = line.removeprefix('error processing uart0:').strip()

    async def _check_after_restart(self) -> None:
        await rosys.sleep(1.0)  # NOTE: the expander runs its startup script after its first answer
        await self._notify_startup_problem()

    async def _check_after_connect(self) -> None:
        await rosys.sleep(5.0)  # NOTE: a reset of the core may restart the expander as well
        await self._notify_startup_problem()

    async def _notify_startup_problem(self) -> None:
        if self.busy or not self.robot_brain.is_ready:
            return
        problem = await self.check_startup()
        if problem is not None:
            rosys.notify(problem, 'warning', log_level=logging.WARNING)

    async def _notify_check(self) -> None:
        problem = await self.check_startup()
        if problem is None:
            rosys.notify(f'{self.name} runs its startup script ({self.local_checksum})', 'positive')
        else:
            rosys.notify(problem, 'warning')

    def developer_ui(self) -> None:
        label = self.name.upper()
        with ui.column():
            ui.label(f'{label} (expander bus)').classes('text-center text-bold')
            ui.label(f'{self.serial.name} @ {self.serial.baud}')
            ui.button(f'Check {label} config', on_click=self._notify_check).props('outline') \
                .tooltip(f'Compare the startup checksum of {label} with the local configuration')
            ui.button(f'Download {label} config', on_click=lambda: ui.download(self.startup.encode(), f'{self.name}.liz')) \
                .props('outline') \
                .tooltip(f'Download the startup script that the next configure writes to {label}')
            ui.button(f'Configure {label}', on_click=self.configure) \
                .tooltip(f'Write the startup script to {label} over the serial bus and restart it')


def _telemetry_line(frame: dict[str, FieldType], interval_ms: int) -> str:
    return f'core.telemetry({", ".join(frame)}, {interval_ms})'
