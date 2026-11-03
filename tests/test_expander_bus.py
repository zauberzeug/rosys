import re
from collections import deque
from dataclasses import dataclass

import pytest
from nicegui import background_tasks

import rosys
from rosys.hardware import (
    BatteryControlHardware,
    BmsHardware,
    ExpanderBusHardware,
    ExpanderHardware,
    RobotBrain,
    RobotHardware,
    SerialHardware,
)
from rosys.hardware.communication import Communication
from rosys.hardware.robot_brain import augment
from rosys.testing import forward

FLASHED = 'expander bus p0 flashed its expander, which restarts as bus node 2'
BOOTSTRAP = ['serial = Serial(3, 1, 460800, 0)', 'bus = SerialBus(serial, 2)']
EXPANDER_STARTUP = '''rdyp = Output(15)
en3 = Output(12)
serial = Serial(26, 27, 460800, 1)
p0 = Expander(serial, 25, 14)
bms = p0.Serial(26, 27, 9600, 2)
bms.unmute()

battery_control_reset = p0.Output(15)
battery_control_status = p0.Input(13)

core.output("core.millis battery_control_status.level")
rdyp.on()
en3.on()
'''
"""What the modules generate behind an ``Expander``, unchanged by the expander bus."""


def checksum(lines: list[str]) -> str:
    startup = ''.join(f'{line}\n' for line in lines)
    return f'{sum(startup.encode()) % 0x10000:04x}'


class CoreSimulation(Communication):
    """The core's console with the expander as bus node 2 behind it, answering like the firmware."""

    def __init__(self) -> None:
        super().__init__()
        self.incoming: deque[str] = deque()
        self.sent: list[str] = []
        self.core_checksum = '0000'
        self.flash_answer = FLASHED
        self.mirrors: dict[str, float] = {}
        self.routines: dict[str, tuple[str, list[str]]] = {}
        """Routines of the core startup that print mirrors, by name: the printed string and the mirrors."""
        self.routines: dict[str, tuple[str, list[str]]] = {}
        """Routines of the core startup that print mirrors, by name: the printed string and the mirrors."""
        self.p0_edit: list[str] = []
        self.p0_saved: list[str] = []
        self.p0_drops: set[str] = set()
        """Startup lines that get lost on the bus."""
        self.p0_noise: list[str] = []
        """Lines that the expander prints before each answer."""

    @classmethod
    def is_possible(cls) -> bool:
        return True

    @property
    def p0_commands(self) -> list[str]:
        return [match.group(1) for line in self.sent if (match := re.fullmatch(r'p0\.send\("(.*)"\)', line))]

    async def send(self, msg: str) -> None:
        line = msg.rsplit('@', 1)[0]
        self.sent.append(line)
        if line == 'core.startup_checksum()':
            self.incoming.append(f'checksum: {self.core_checksum}')
        elif line == 'p0.flash()':
            self.incoming.append(self.flash_answer)
            if self.flash_answer == FLASHED:
                self.p0_edit = list(BOOTSTRAP)
                self.p0_saved = list(BOOTSTRAP)
                self.incoming.append('bus[2]: Ready.')
        elif line == 'p0.restart()':
            self.p0_edit = list(self.p0_saved)
            self.incoming.append('bus[2]: Ready.')
        elif match := re.fullmatch(r'p0\.send\("(.*)"\)', line):
            self._run_on_p0(match.group(1).replace('\\"', '"').replace('%%', '%'))
        elif match := re.fullmatch(r'(\w+)\(\)', line):
            text, names = self.routines[match.group(1)]
            values = [self.mirrors[name] for name in names]
            self.incoming.append(f'"{text}" ' + ' '.join(f'{value:f}' if isinstance(value, float) else str(value)
                                                         for value in values))

    def _run_on_p0(self, command: str) -> None:
        self.incoming.extend(f'bus[2]: {noise}' for noise in self.p0_noise)
        if command == '!-':
            self.p0_edit = []
        elif command.startswith('!+'):
            if command[2:] not in self.p0_drops:
                self.p0_edit.append(command[2:])
        elif command == '!.':
            self.p0_saved = list(self.p0_edit)
        elif command == 'core.startup_checksum()':
            self.incoming.append(f'bus[2]: checksum: {checksum(self.p0_edit)}')
        elif command == 'core.version()':
            self.incoming.append('bus[2]: version: lizard v0.16.0-12-gabcdef0')
        elif match := re.fullmatch(r'core\.get_pin_status\((\d+)\)', command):
            self.incoming.append(f'bus[2]: GPIO_Status[{match.group(1)}]| Level: 1| InputEn: 1| OutputEn: 0| '
                                 'OpenDrain: 0| Pullup: 1| Pulldown: 0| DriveStrength: 2| SleepSel: 0')
        elif command == 'bms.cell_voltages()':
            self.incoming.append('bus[2]: bms cell voltages: 3.650 3.657')
        elif command == 'bms.hardware_version()':
            self.incoming.append('bus[2]: bms hardware version: OH17SA01-L17S-100A')

    async def read(self) -> str | None:
        return augment(self.incoming.popleft()) if self.incoming else None


@dataclass(kw_only=True)
class Robot:
    core: CoreSimulation
    brain: RobotBrain
    p0: ExpanderBusHardware
    bms: BmsHardware
    battery: BatteryControlHardware
    hardware: RobotHardware
    notifications: list[str]


def build(core: CoreSimulation) -> tuple[RobotBrain, ExpanderBusHardware, BmsHardware, BatteryControlHardware,
                                         RobotHardware]:
    brain = RobotBrain(core, enable_esp_on_startup=False)
    serial = SerialHardware(brain, baud=460800)
    p0 = ExpanderBusHardware(brain, serial=serial)
    bms = BmsHardware(brain, expander=p0, num=2)
    battery = BatteryControlHardware(brain, expander=p0, name='battery_control')
    hardware = RobotHardware([serial, p0, bms, battery], brain)
    return brain, p0, bms, battery, hardware


@pytest.fixture
async def robot(rosys_integration: None) -> Robot:
    """F30's P0 modules on an expander bus, with the core and the expander running their startup scripts."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    core = CoreSimulation()
    brain, p0, bms, battery, hardware = build(core)
    core.core_checksum = checksum(brain.lizard_code.splitlines())
    for match in re.finditer(r'let (\w+) do core\.print\("(\w+)", (.*)\); end', brain.lizard_code):
        core.routines[match.group(1)] = (match.group(2), match.group(3).split(', '))
    core.p0_edit = core.p0_saved = p0.startup.splitlines()
    bms.state.last_update = 1e9  # NOTE: no BMS readings, unless a test asks for them
    for millis in (1000, 1010):
        core.incoming.append(f'core {millis} 1')
        await forward(seconds=1.0)
    await forward(seconds=6.0)  # NOTE: the startup checks after connecting
    assert brain.is_ready
    assert p0.runs_startup
    core.sent.clear()
    return Robot(core=core, brain=brain, p0=p0, bms=bms, battery=battery, hardware=hardware,
                 notifications=notifications)


async def run(coroutine):
    task = background_tasks.create(coroutine, name='test')
    await forward(until=task.done, timeout=300)
    return task.result()


async def test_an_expander_keeps_its_startup(rosys_integration: None) -> None:
    brain = RobotBrain(CoreSimulation(), enable_esp_on_startup=False)
    serial = SerialHardware(brain, baud=460800)
    p0 = ExpanderHardware(brain, serial=serial)
    modules = [serial, p0, BmsHardware(brain, expander=p0, num=2),
               BatteryControlHardware(brain, expander=p0, name='battery_control')]
    RobotHardware(modules, brain)
    assert brain.lizard_code == EXPANDER_STARTUP


async def test_the_core_mirrors_the_values_of_the_expander(robot: Robot) -> None:
    assert robot.brain.lizard_code.splitlines() == [
        'rdyp = Output(15)',
        'en3 = Output(12)',
        'serial = Serial(26, 27, 460800, 1)',
        'p0 = ExpanderBus(serial, 25, 14)',
        'float p0.bms.voltage = 0',
        'float p0.bms.current = 0',
        'int p0.bms.percentage = 0',
        'float p0.bms.temperature = 0',
        'int p0.bms.protections = 0',
        'int p0.bms.last_message_age = 0',
        'int p0.core.millis = 0',
        'let bms_read do core.print("bms", p0.bms.voltage, p0.bms.current, p0.bms.percentage, p0.bms.temperature, '
        'p0.bms.protections, p0.bms.last_message_age, p0.core.millis); end',
        'int p0.battery_control_status.level = 0',
        'core.output("core.millis p0.battery_control_status.level")',
        'rdyp.on()',
        'en3.on()',
    ]
    robot.core.incoming.append('core 1020 0')
    await forward(seconds=0.1)
    assert robot.battery.status is False


async def test_the_expander_runs_the_modules_and_sends_their_values(robot: Robot) -> None:
    assert robot.p0.startup.splitlines() == [
        *BOOTSTRAP,  # NOTE: what flash() writes, so that a later error reaches the core
        'bms_serial = Serial(26, 27, 9600, 2)',
        'bms = JbdBms(bms_serial)',
        'battery_control_reset = Output(15)',
        'battery_control_status = Input(13)',
        'int battery_control_release_t = 0',
        'let battery_control_release do battery_control_release_t = core.millis; battery_control_reset.on(); '
        'await core.millis > battery_control_release_t + 500; battery_control_reset.off(); end',
        'core.telemetry(battery_control_status.level, 100)',
        'core.telemetry(bms.voltage, bms.current, bms.percentage, bms.temperature, bms.protections, '
        'bms.last_message_age, core.millis, 1000)',
    ]
    assert robot.p0.local_checksum == checksum(robot.p0.startup.splitlines())


async def test_the_bms_reads_the_mirrors_of_new_answers(robot: Robot) -> None:
    def mirror(voltage: float, age: int, millis: int) -> None:
        robot.core.mirrors = {'p0.bms.voltage': voltage, 'p0.bms.current': -1.5, 'p0.bms.percentage': 57,
                              'p0.bms.temperature': 24.85, 'p0.bms.protections': 0,
                              'p0.bms.last_message_age': age, 'p0.core.millis': millis}
    states: list[float] = []
    robot.bms.STATE_UPDATED.subscribe(lambda state: states.append(state.voltage))

    mirror(0.0, 3000, 3000)  # NOTE: the BMS has not answered yet
    robot.bms.state.last_update = -10
    await forward(seconds=1.5)
    assert states == []
    assert robot.core.sent[-1] == 'bms_read()'

    mirror(26.5, 300, 5000)
    await forward(seconds=1.0)
    assert states == [26.5]
    assert (robot.bms.state.percentage, robot.bms.state.current, robot.bms.state.temperature) == (57, -1.5, 24.85)
    assert robot.bms.state.is_charging is False
    assert robot.bms.state.last_update == pytest.approx(rosys.time() - 0.3, abs=1.0)

    robot.bms.state.last_update = -10
    mirror(26.4, 1305, 6005)  # NOTE: the same answer, e.g. since the BMS or the expander fell silent
    await forward(seconds=1.0)
    assert states == [26.5]

    mirror(26.4, 200, 7000)
    await forward(seconds=1.0)
    assert states == [26.5, 26.4]


async def test_the_bms_asks_the_expander_for_cell_voltages_and_its_version(robot: Robot) -> None:
    await run(robot.bms.request_cell_voltages())
    await run(robot.bms.request_hardware_version())
    await forward(seconds=0.1)
    assert robot.core.p0_commands == ['bms.cell_voltages()', 'bms.hardware_version()']
    assert robot.bms.raw_data == {'voltages': [3.65, 3.657], 'hardware version': 'OH17SA01-L17S-100A'}

    robot.core.mirrors = {'p0.bms.voltage': 26.5, 'p0.bms.current': 0.0, 'p0.bms.percentage': 57,
                          'p0.bms.temperature': 24.85, 'p0.bms.protections': 0, 'p0.bms.last_message_age': 300,
                          'p0.core.millis': 5000}
    robot.bms.state.last_update = -10
    await forward(seconds=1.5)
    assert robot.bms.raw_data['voltages'] == [3.65, 3.657], 'a reading keeps the cell voltages'
    assert robot.bms.raw_data['total voltage'] == 26.5


async def test_the_expander_times_the_relay_pulse(robot: Robot) -> None:
    await run(robot.battery.release_battery_relay())
    assert robot.core.p0_commands == ['battery_control_release()']

    robot.core.sent.clear()
    await run(robot.battery.relay_on())
    await run(robot.battery.relay_off())
    assert robot.core.p0_commands == ['battery_control_reset.on()', 'battery_control_reset.off()']


async def test_commands_wait_for_the_startup_of_the_expander(robot: Robot) -> None:
    robot.core.p0_edit = robot.core.p0_saved = list(BOOTSTRAP)  # NOTE: e.g. right after flashing
    robot.core.incoming.append('bus[2]: Ready.')
    await forward(seconds=0.1)
    assert not robot.p0.runs_startup
    await run(robot.battery.release_battery_relay())
    assert 'battery_control_release()' not in robot.core.p0_commands
    assert any('"battery_control_release()" was not sent' in message for message in robot.notifications)

    robot.core.p0_edit = robot.core.p0_saved = robot.p0.startup.splitlines()
    await run(robot.bms.request_cell_voltages())
    assert robot.core.p0_commands[-1] == 'bms.cell_voltages()'


async def test_a_restarted_expander_is_checked(robot: Robot) -> None:
    robot.core.p0_edit = robot.core.p0_saved = ['old = Output(2)']
    robot.core.incoming.append('bus[2]: Ready.')
    await forward(seconds=3.0)
    assert any('please configure p0' in message for message in robot.notifications)

    robot.core.incoming.append('bus[2]: error: startup skipped after 2 failed boots (panic)')
    await forward(seconds=0.1)
    assert robot.notifications[-1] == 'p0: error: startup skipped after 2 failed boots (panic)'


async def test_configure_verifies_before_saving_and_after_restarting(robot: Robot) -> None:
    robot.core.p0_saved = ['old = Output(2)']
    assert await run(robot.p0.configure())
    lines = robot.p0.startup.splitlines()
    assert robot.core.p0_saved == lines
    assert robot.core.p0_commands == ['!-', *(f'!+{line}' for line in lines), 'core.startup_checksum()', '!.',
                                      'core.startup_checksum()', 'core.startup_checksum()']
    assert robot.core.sent.index('p0.restart()') > robot.core.sent.index('p0.send("!.")')
    assert robot.p0.runs_startup


async def test_configure_saves_nothing_after_a_lost_line(robot: Robot) -> None:
    robot.core.p0_saved = ['old = Output(2)']
    robot.core.p0_drops = {'bms = JbdBms(bms_serial)'}
    assert not await run(robot.p0.configure())
    assert robot.core.p0_saved == ['old = Output(2)']
    assert '!.' not in robot.core.p0_commands
    assert 'p0.restart()' in robot.core.sent, 'a restart leaves the edited script for the saved one'
    assert robot.core.p0_edit == ['old = Output(2)']


async def test_configure_and_flash_refuse_to_run_twice(robot: Robot) -> None:
    first = background_tasks.create(robot.p0.configure(), name='first')
    await forward(seconds=0.01)
    assert not await run(robot.p0.configure())
    assert not await run(robot.p0.flash())
    await forward(until=first.done)
    assert first.result()
    assert robot.core.p0_commands.count('!-') == 1


async def test_a_skipped_startup_is_no_running_startup(robot: Robot) -> None:
    robot.core.incoming.append('bus[2]: Ready.')
    robot.core.incoming.append('bus[2]: error: startup skipped after 2 failed boots (panic)')
    await forward(seconds=3.0)  # NOTE: the check after the restart finds the saved script, which did not run
    assert not robot.p0.runs_startup
    await run(robot.battery.release_battery_relay())
    assert 'battery_control_release()' not in robot.core.p0_commands


async def test_other_peers_2_keep_their_lines(robot: Robot) -> None:
    lines: list[str] = []
    robot.battery.message_hooks['bus[2]:'] = lines.append  # NOTE: e.g. a plexus as peer 2 of another serial bus
    robot.core.incoming.append('bus[2]: checksum: abcd')
    await forward(seconds=0.1)
    assert lines == ['bus[2]: checksum: abcd']


async def test_a_failed_request_leaves_no_reply_behind(robot: Robot) -> None:
    with pytest.raises(ValueError):
        await robot.p0.request('x' * 300, 'checksum:')
    robot.core.incoming.append('bus[2]: checksum: abcd')
    await forward(seconds=0.1)
    assert await run(robot.p0.read_checksum()) == robot.p0.local_checksum


async def test_flash_copies_the_firmware_and_configures_the_expander(robot: Robot) -> None:
    await run(robot.brain.lizard_firmware.flash_p0())
    sent = robot.core.sent
    assert sent.index('p0.flash()') < sent.index('p0.send("!-")')
    assert robot.core.p0_saved == robot.p0.startup.splitlines()
    assert 'core.restart()' not in sent  # NOTE: the core keeps running, unlike after an expander's flash
    assert robot.brain.lizard_firmware.p0_version == '0.16.0'
    assert robot.notifications[-1] == 'Finished.'


async def test_flash_needs_the_core_startup_with_the_expander_bus(robot: Robot) -> None:
    robot.core.core_checksum = 'beef'  # NOTE: e.g. the core still runs an expander
    await run(robot.brain.lizard_firmware.flash_p0())
    assert 'p0.flash()' not in robot.core.sent


async def test_a_failed_flash_writes_no_startup(robot: Robot) -> None:
    robot.core.flash_answer = 'error processing uart0: could not flash the expander of expander bus "p0"'
    assert not await run(robot.p0.flash(timeout=5))
    assert robot.core.p0_commands == []
    assert robot.notifications[-1] == 'Flashing p0 failed: could not flash the expander of expander bus "p0"'


async def test_the_version_and_pins_of_the_expander_come_over_the_bus(robot: Robot) -> None:
    robot.core.p0_noise = ['warning: something', 'GPIO_Status[2]| Level: 0']  # NOTE: other lines before the answer
    await run(robot.brain.lizard_firmware.read_p0_version())
    assert (robot.brain.lizard_firmware.p0_project, robot.brain.lizard_firmware.p0_version) == ('lizard', '0.16.0')

    robot.core.sent.clear()
    assert await run(robot.brain.esp_pins_p0.get_pin_level(15)) is True
    assert robot.core.p0_commands == ['core.get_pin_status(15)']


async def test_the_expander_refuses_what_its_startup_or_frames_cannot_carry(rosys_integration: None) -> None:
    brain = RobotBrain(CoreSimulation(), enable_esp_on_startup=False)
    p0 = ExpanderBusHardware(brain, serial=SerialHardware(brain))
    assert p0.add_lizard_code('a = Input(4)', [('a.level', 'int')]) == 'int p0.a.level = 0'
    with pytest.raises(ValueError, match='mirrored already'):
        p0.add_lizard_code('', [('a.level', 'int')], interval=1.0)
    with pytest.raises(ValueError, match='exceeds a bus message'):
        p0.add_lizard_code('x = "' + 'y' * 250 + '"')
    with pytest.raises(ValueError, match='exceed one telemetry frame'):
        p0.add_lizard_code('', [(f'f{index}', 'float') for index in range(46)])
    with pytest.raises(ValueError, match='at most 8 telemetry frames'):
        for index in range(8):
            p0.add_lizard_code('', [(f'g{index}', 'bool')], interval=index + 1)
    assert p0.startup.splitlines()[2:4] == ['a = Input(4)', 'core.telemetry(a.level, 100)']
