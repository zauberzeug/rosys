import asyncio
import base64
import binascii
import struct
from collections.abc import Sequence

import pytest
from test_robot_brain import MISMATCH_MESSAGE, CommunicationSimulation  # pylint: disable=wrong-import-order

import rosys
from rosys.hardware import ModuleHardware, RobotBrain, RobotHardware
from rosys.hardware.core_telemetry import LINE_LIMIT, CoreTelemetry
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


def planned_frames(code: str) -> list[list[str]]:
    return [line.removeprefix('core.telemetry(').removesuffix(')').split(', ') for line in code.splitlines()]


def test_the_frame_helpers_match_lizards_documented_example() -> None:
    fields = [('motor.position', 1.234), ('motor.axis_state', 8), ('motor.enabled', True)]
    assert layout_lines(1, fields)[0] == '__LAYOUT__v1 1.0/3 bf motor.position:f'
    assert frame_line(1, fields, seq=7, millis=123456) == '~Ab8HQOIBALbznT8IAAAAAY/t'


def test_core_fields_go_into_one_frame_once_and_without_their_precision() -> None:
    code = CoreTelemetry(lambda: None).lizard_code(['wheels.linear_speed:4', 'wheels.locked',
                                                    'p2t.delta_2.angle_left:3', 'p2_ms', 'wheels.locked'])
    assert code == 'core.telemetry(wheels.linear_speed, wheels.locked, p2t.delta_2.angle_left, p2_ms)\n'


def test_core_fields_beyond_a_frame_split_into_several_frames() -> None:
    code = CoreTelemetry(lambda: None).lizard_code([f'module.field_{index}:2' for index in range(100)])
    assert [len(frame) for frame in planned_frames(code)] == [45, 45, 10]  # 45 fields of 4 bytes fill 180 bytes


def test_core_fields_beyond_a_startup_script_line_split_into_several_frames() -> None:
    code = CoreTelemetry(lambda: None).lizard_code([f'module.{"x" * 50}_{index}:2' for index in range(40)])
    lines = code.splitlines()
    assert len(lines) == 2  # 40 fields of 4 bytes fit into one frame, but their names not into one line
    assert all(len(line) <= LINE_LIMIT for line in lines)


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


def test_the_frames_of_one_step_are_reassembled_by_millis() -> None:
    telemetry = CoreTelemetry(lambda: None)
    first, second = planned_frames(telemetry.lizard_code([f'm.f{index}:1' for index in range(50)]))
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
