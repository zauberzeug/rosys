import asyncio
import base64
import binascii
import struct
from collections.abc import Sequence

import pytest
from nicegui import background_tasks
from test_robot_brain import (  # pylint: disable=wrong-import-order
    MISMATCH_MESSAGE,
    CommunicationSimulation,
    matching_checksum,
)

import rosys
from rosys.hardware import ModuleHardware, RobotBrain, RobotHardware, robot_brain
from rosys.hardware.core_telemetry import LINE_LIMIT, OUTDATED_AFTER_STEPS, CoreTelemetry
from rosys.testing import forward

TYPE_LETTERS = {bool: '?', int: 'i', float: 'f'}

Fields = Sequence[tuple[str, object]]  # name and value, typed by the value


def crc(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xffff)  # CRC-16/CCITT-FALSE


def layout(fields: Fields) -> tuple[list[str], int]:
    """The fields as Lizard describes them in layout lines, and the layout ID of their frame."""
    described = [f'{name}:{TYPE_LETTERS[type(value)]}' for name, value in fields]
    return described, crc(' '.join(described).encode()) & 0xff


def layout_lines(frame_id: int, fields: Fields) -> list[str]:
    described, layout_id = layout(fields)
    return [f'__LAYOUT__v1 {frame_id}.{index}/{len(fields)} {layout_id:02x} {field}'
            for index, field in enumerate(described)]


def frame_line(frame_id: int, fields: Fields, *, seq: int = 0, millis: int = 0) -> str:
    """A frame line as Lizard encodes it: numbers first, then the bools as bits."""
    _, layout_id = layout(fields)
    body = struct.pack('<BBBI', frame_id, layout_id, seq, millis)
    bools = [value for _, value in fields if isinstance(value, bool)]
    for _, value in fields:
        if not isinstance(value, bool):
            body += struct.pack('<i' if isinstance(value, int) else '<f', value)
    bits = bytearray((len(bools) + 7) // 8)
    for index, value in enumerate(bools):
        bits[index // 8] |= value << index % 8
    body += bytes(bits)
    body += crc(body).to_bytes(2, 'little')
    return '~' + base64.b64encode(body).decode()


def calls(code: str) -> list[list[str]]:
    return [line.removeprefix('core.telemetry(').removesuffix(')').split(', ') for line in code.splitlines()]


def test_the_frame_helpers_match_lizards_documented_example() -> None:
    fields = [('motor.position', 1.234), ('motor.axis_state', 8), ('motor.enabled', True)]
    assert layout_lines(1, fields)[0] == '__LAYOUT__v1 1.0/3 bf motor.position:f'
    assert frame_line(1, fields, seq=7, millis=123456) == '~Ab8HQOIBALbznT8IAAAAAY/t'


def test_core_fields_go_into_one_call_once_and_without_their_precision() -> None:
    code = CoreTelemetry(lambda: None).lizard_code(['wheels.linear_speed:4', 'wheels.locked',
                                                    'p2t.delta_2.angle_left:3', 'p2_ms', 'wheels.locked'])
    assert code == 'core.telemetry(wheels.linear_speed, wheels.locked, p2t.delta_2.angle_left, p2_ms)\n'


def test_core_fields_beyond_a_frame_stay_in_one_call_which_lizard_splits() -> None:
    code = CoreTelemetry(lambda: None).lizard_code([f'module.field_{index}:2' for index in range(100)])
    assert [len(call) for call in calls(code)] == [100]  # 400 bytes of values, which Lizard sends in three frames


def test_core_fields_beyond_a_startup_script_line_go_into_several_calls() -> None:
    fields = [f'module.{"x" * 50}_{index}:2' for index in range(40)]
    code = CoreTelemetry(lambda: None).lizard_code(fields)
    lines = code.splitlines()
    assert len(lines) == 2  # the names of the 40 fields do not fit into one line
    assert all(len(line) <= LINE_LIMIT for line in lines)
    assert [name for call in calls(code) for name in call] == [field.removesuffix(':2') for field in fields]


def test_core_fields_must_be_properties_or_variables() -> None:
    telemetry = CoreTelemetry(lambda: None)
    with pytest.raises(ValueError):
        telemetry.lizard_code(['wheels.speed * 2'])
    with pytest.raises(ValueError):
        telemetry.lizard_code(['wheels.speed:x'])


def test_a_frame_becomes_the_text_line_core_output_would_print() -> None:
    telemetry = CoreTelemetry(lambda: None)
    telemetry.lizard_code(['wheels.linear_speed:4', 'wheels.locked', 'p2t.delta_2.state', 'wheels.distance:1',
                           'wheels.locked', 'heading'])
    fields = [('wheels.linear_speed', -0.25), ('wheels.locked', True), ('p2t.delta_2.state', 3),
              ('wheels.distance', 12.34), ('heading', 1.75)]
    for line in layout_lines(1, fields):
        assert telemetry.translate(line) == ''
    # the millis come from the frame header, wheels.locked is repeated, and a float without precision prints like "%.0f"
    assert telemetry.translate(frame_line(1, fields, millis=40914)) == 'core 40914 -0.2500 true 3 12.3 true 2'


def test_without_core_fields_a_frame_of_millis_still_marks_each_step() -> None:
    telemetry = CoreTelemetry(lambda: None)
    assert telemetry.lizard_code([]) == 'core.telemetry(core.millis)\n'
    fields = [('core.millis', 7)]
    for line in layout_lines(1, fields):
        telemetry.translate(line)
    assert telemetry.translate(frame_line(1, fields, millis=8)) == 'core 8'


def test_other_lines_pass_unchanged() -> None:
    telemetry = CoreTelemetry(lambda: None)
    assert telemetry.translate('core 5 true') == 'core 5 true'
    assert telemetry.translate('bus[2]: ~AbC=') == 'bus[2]: ~AbC='  # a peer's frame is not the core's


@pytest.mark.parametrize('split', [45, 20], ids=['45 + 5 fields', '20 + 30 fields'])
def test_the_frames_of_one_step_are_reassembled_by_millis(split: int) -> None:
    telemetry = CoreTelemetry(lambda: None)
    names = [f'm.f{index}' for index in range(50)]
    telemetry.lizard_code([f'{name}:1' for name in names])
    first, second = names[:split], names[split:]  # Lizard decides where a frame ends; the layouts tell
    for line in layout_lines(1, [(name, 0) for name in first]) + layout_lines(2, [(name, 0) for name in second]):
        telemetry.translate(line)

    def step(millis: int) -> tuple[str, str]:
        values = {f'm.f{index}': index + millis for index in range(50)}  # ints stand in for the fields
        return (frame_line(1, [(name, values[name]) for name in first], millis=millis),
                frame_line(2, [(name, values[name]) for name in second], millis=millis))

    def text_line(millis: int) -> str:
        return f'core {millis} ' + ' '.join(str(index + millis) for index in range(50))

    a, b = step(100)
    assert telemetry.translate(a) == ''
    assert telemetry.translate(b) == text_line(100)
    lost, _ = step(110)
    assert telemetry.translate(lost) == ''  # its second frame never arrives
    a, b = step(120)
    assert telemetry.translate(a) == ''
    assert telemetry.translate(b) == text_line(120)


def test_missing_layouts_are_requested_at_most_every_two_seconds(rosys_integration: None) -> None:
    requests: list[float] = []
    telemetry = CoreTelemetry(lambda: requests.append(rosys.time()))
    telemetry.lizard_code(['wheels.locked'])
    fields = [('wheels.locked', False)]
    for now in (0.0, 0.5, 1.0, 1.99, 2.0, 3.0, 4.1):
        rosys.set_time(now)
        assert telemetry.translate(frame_line(1, fields)) == ''
    assert requests == [0.0, 2.0, 4.1]
    for line in layout_lines(1, fields):
        telemetry.translate(line)
    assert telemetry.translate(frame_line(1, fields, millis=5)) == 'core 5 false'
    assert requests == [0.0, 2.0, 4.1]


def test_a_frame_of_an_outdated_startup_script_is_dropped_with_a_notification(rosys_integration: None) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    telemetry = CoreTelemetry(lambda: None)
    telemetry.lizard_code(['wheels.locked'])
    fields = [('wheels.speed', 0.5)]
    for line in layout_lines(1, fields):
        telemetry.translate(line)
    assert telemetry.translate(frame_line(1, fields)) == ''
    assert telemetry.translate(frame_line(1, fields)) == ''
    assert [MISMATCH_MESSAGE in message for message in notifications] == [True]  # once per 30 s


def test_layouts_that_are_still_coming_are_not_requested_again(rosys_integration: None) -> None:
    requests: list[float] = []
    telemetry = CoreTelemetry(lambda: requests.append(rosys.time()))
    telemetry.lizard_code(['m.a', 'm.b'])
    fields = [('m.a', 1), ('m.b', 2)]
    rosys.set_time(0.0)
    assert telemetry.translate(frame_line(1, fields)) == ''
    rosys.set_time(1.9)
    telemetry.translate(layout_lines(1, fields)[0])  # the first of two layout lines
    rosys.set_time(2.5)
    assert telemetry.translate(frame_line(1, fields)) == ''
    assert requests == [0.0]


def test_frames_that_keep_lacking_fields_count_as_an_outdated_startup_script(rosys_integration: None) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    telemetry = CoreTelemetry(lambda: None)
    telemetry.lizard_code(['m.a', 'm.b'])
    old_fields = [('m.a', 1)]  # the core still runs a startup script without m.b
    for line in layout_lines(1, old_fields):
        telemetry.translate(line)
    for step in range(OUTDATED_AFTER_STEPS):
        assert telemetry.translate(frame_line(1, old_fields, millis=10 * step)) == ''
    assert not notifications  # a lost frame or layout lines on their way leave steps incomplete as well
    assert telemetry.translate(frame_line(1, old_fields, millis=10 * OUTDATED_AFTER_STEPS)) == ''
    assert [MISMATCH_MESSAGE in message for message in notifications] == [True]


def test_malformed_telemetry_lines_are_dropped() -> None:
    requests: list[None] = []
    telemetry = CoreTelemetry(lambda: requests.append(None))
    telemetry.lizard_code(['wheels.locked'])
    good = frame_line(1, [('wheels.locked', False)])
    zero_crc = good[:-4] + 'AAA='  # the last Base64 quantum holds the two CRC bytes
    assert telemetry.translate(zero_crc) == ''
    assert telemetry.translate('~not base64') == ''
    assert requests == []


class _Module(ModuleHardware):
    def __init__(self, robot_brain: RobotBrain) -> None:
        super().__init__(robot_brain=robot_brain, lizard_code='m = Module()',
                         core_message_fields=['m.speed:2', 'm.flag', 'count'])
        self.received: list[list[str]] = []

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.received.append([words.pop(0) for _ in range(3)])


def _module_fields(step: int) -> Fields:
    return [('m.speed', 0.5 * step), ('m.flag', step % 2 == 1), ('count', step)]


async def test_the_robot_brain_turns_core_frames_into_core_messages(rosys_integration: None) -> None:
    communication = CommunicationSimulation()
    brain = RobotBrain(communication, enable_esp_on_startup=False, telemetry_frames=True)
    module = _Module(brain)
    robot = RobotHardware([module], brain, rdyp_pin=None, en3_pin=None)
    assert 'core.output' not in brain.lizard_code
    assert brain.lizard_code.endswith('core.telemetry(m.speed, m.flag, count)\n')
    millis: list[float] = []
    brain.CORE_MESSAGE_RECEIVED.subscribe(millis.append)

    def frame(step: int) -> str:
        return frame_line(1, _module_fields(step), millis=1000 + 10 * step)

    communication.incoming.extend([frame(0), frame(1)])
    assert await brain.read_lines() == []  # without layouts no core message, and no line before the brain is ready
    await asyncio.sleep(0)  # let the background request go out
    assert communication.sent == ['core.telemetry_info()']  # once, not for the second frame
    assert millis == []

    communication.incoming.extend([*layout_lines(1, _module_fields(2)), frame(2)])
    await robot.update()
    assert millis == [1020.0]
    assert not brain.is_ready  # the first core message only sets the clock offset
    communication.incoming.append(frame(3))
    await robot.update()
    assert brain.is_ready
    assert module.received == [['1.50', 'true', '3']]

    communication.incoming.extend([frame(4), 'core 9 9 9 9'])  # a text line still counts
    await robot.update()
    assert module.received[1:] == [['2.00', 'false', '4'], ['9', '9', '9']]


async def test_without_telemetry_frames_the_core_prints_the_text_line(rosys_integration: None) -> None:
    brain = RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False)
    _robot = RobotHardware([_Module(brain)], brain, rdyp_pin=None, en3_pin=None)  # NOTE: keep alive for its updates
    assert brain.lizard_code.endswith('core.output("core.millis m.speed:2 m.flag count")\n')


async def test_without_telemetry_frames_core_frames_ask_to_configure(rosys_integration: None) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = CommunicationSimulation()
    brain = RobotBrain(communication, enable_esp_on_startup=False)
    _robot = RobotHardware([_Module(brain)], brain, rdyp_pin=None, en3_pin=None)  # NOTE: keep alive for its updates
    communication.incoming.append(frame_line(1, _module_fields(1)))  # the core still runs a frames startup script
    await forward(seconds=1.0)
    assert communication.sent == ['core.telemetry_info()']
    communication.incoming.extend([*layout_lines(1, _module_fields(2)), frame_line(1, _module_fields(2))])
    await forward(seconds=1.0)
    assert any(MISMATCH_MESSAGE in message for message in notifications)


LIZARD_0_15 = ['project name: lizard', 'version: v0.15.1']
LIZARD_ZZ_ON_0_15 = ['project name: lizard-zz', 'version: v0.2.0']
LIZARD_ZZ_ON_0_16 = ['project name: lizard-zz', 'project version: v0.3.0+1a2b3c4d', 'lizard version: v0.16.0']
LIZARD_BEFORE_0_16 = ['project name: lizard', 'version: v0.15.1-40-gea96593']  # a build between two releases


class _Core(CommunicationSimulation):
    """A core that answers ``core.info()``, unless it has none, and rejects ``core.telemetry_info()`` unless it knows
    telemetry."""

    def __init__(self, info: list[str] | None, *, knows_telemetry: bool) -> None:
        super().__init__()
        self.info = [] if info is None else [*info, 'compile time: Nov 13 2026, 17:05:21', 'idf version: v5.3.6']
        self.knows_telemetry = knows_telemetry

    async def send(self, msg: str) -> None:
        await super().send(msg)
        if self.sent[-1] == 'core.info()':
            self.incoming.extend(self.info)
        if self.sent[-1] == 'core.telemetry_info()' and not self.knows_telemetry:
            self.incoming.append('error processing uart0: unknown method "core.telemetry_info"')


@pytest.mark.parametrize('info, knows_telemetry, refusal, asked', [
    (LIZARD_0_15, False, 'needs Lizard v0.16.0 or newer, but the firmware is built from Lizard v0.15.1', False),
    (LIZARD_ZZ_ON_0_15, False, 'need Lizard v0.16.0 or newer, but the firmware does not know them', True),
    (LIZARD_ZZ_ON_0_16, True, None, False),
    (LIZARD_BEFORE_0_16, True, None, True),
], ids=['Lizard 0.15.1', 'lizard-zz on Lizard 0.15.1', 'lizard-zz on Lizard 0.16.0', 'build before 0.16.0'])
async def test_configure_refuses_telemetry_frames_on_a_core_that_lacks_them(rosys_integration: None,
                                                                            info: list[str], knows_telemetry: bool,
                                                                            refusal: str | None, asked: bool) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = _Core(info, knows_telemetry=knows_telemetry)
    brain = RobotBrain(communication, enable_esp_on_startup=False, telemetry_frames=True)
    _robot = RobotHardware([_Module(brain)], brain, rdyp_pin=None, en3_pin=None)  # NOTE: keep alive for its updates
    communication.startup_checksum = matching_checksum(brain)
    task = background_tasks.create(brain.configure(), name='configure')
    await forward(seconds=25.0)
    assert task.result() == (refusal is None)
    assert ('core.telemetry_info()' in communication.sent) == asked  # only where the version leaves it open
    failures = [notification for notification in notifications if 'failed' in notification]
    if refusal is None:
        assert not failures
        assert communication.sent.count('!.') == 1
    else:
        assert [refusal in failure for failure in failures] == [True]
        assert not any(line.startswith('!') for line in communication.sent), 'the core must keep its startup script'
        assert 'core.restart()' not in communication.sent


async def test_configure_goes_on_without_an_answer_to_core_info(rosys_integration: None,
                                                                monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(robot_brain, 'FIRMWARE_VERSION_TIMEOUT', 0.05)
    communication = _Core(None, knows_telemetry=True)
    brain = RobotBrain(communication, enable_esp_on_startup=False, telemetry_frames=True)
    _robot = RobotHardware([_Module(brain)], brain, rdyp_pin=None, en3_pin=None)  # NOTE: keep alive for its updates
    communication.startup_checksum = matching_checksum(brain)
    task = background_tasks.create(brain.configure(), name='configure')
    await asyncio.sleep(0.1)  # NOTE: lizard-host waits for the answer to core.info() on the event loop's clock
    await forward(seconds=25.0)
    assert task.result() is True
    assert 'core.telemetry_info()' in communication.sent


async def test_configure_without_telemetry_frames_does_not_ask_for_the_firmware(rosys_integration: None) -> None:
    communication = _Core(LIZARD_0_15, knows_telemetry=False)
    brain = RobotBrain(communication, enable_esp_on_startup=False)
    _robot = RobotHardware([_Module(brain)], brain, rdyp_pin=None, en3_pin=None)  # NOTE: keep alive for its updates
    communication.startup_checksum = matching_checksum(brain)
    task = background_tasks.create(brain.configure(), name='configure')
    await forward(seconds=25.0)
    assert task.result() is True
    assert 'core.info()' not in communication.sent
    assert 'core.telemetry_info()' not in communication.sent
