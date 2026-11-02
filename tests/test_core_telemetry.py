"""The core's telemetry as Lizard telemetry frames instead of the ``core.output`` text line."""
from __future__ import annotations

import asyncio
import base64
import binascii
import struct

import pytest

from rosys.hardware import ModuleHardware, RobotBrain, RobotHardware
from rosys.hardware.communication import Communication
from rosys.hardware.core_telemetry import CoreTelemetry
from rosys.hardware.robot_brain import augment

TYPE_LETTERS = {bool: '?', int: 'i', float: 'f'}


def crc(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xffff)  # CRC-16/CCITT-FALSE


def layout_lines(frame_id: int, fields: list[tuple[str, object]]) -> list[str]:
    """The layout lines Lizard sends for a frame of ``(name, value)`` fields, typed by the values."""
    described = [f'{name}:{TYPE_LETTERS[type(value)]}' for name, value in fields]
    layout_id = crc(' '.join(described).encode()) & 0xff
    return [f'__LAYOUT__v1 {frame_id}.{index}/{len(fields)} {layout_id:02x} {field}'
            for index, field in enumerate(described)]


def frame_line(frame_id: int, fields: list[tuple[str, object]], *, seq: int = 0, millis: int = 0) -> str:
    """A frame line as Lizard encodes it: numbers first, then the bools as bits."""
    described = [f'{name}:{TYPE_LETTERS[type(value)]}' for name, value in fields]
    layout_id = crc(' '.join(described).encode()) & 0xff
    body = struct.pack('<BBBI', frame_id, layout_id, seq, millis)
    bools = [value for _, value in fields if isinstance(value, bool)]
    for _, value in fields:
        if isinstance(value, bool):
            continue
        body += struct.pack('<i' if isinstance(value, int) else '<f', value)
    bits = bytearray((len(bools) + 7) // 8)
    for index, value in enumerate(bools):
        bits[index // 8] |= value << index % 8
    body += bytes(bits)
    body += crc(body).to_bytes(2, 'little')
    return '~' + base64.b64encode(body).decode()


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_the_encoder_matches_the_documented_example() -> None:
    fields = [('motor.position', 1.234), ('motor.axis_state', 8), ('motor.enabled', True)]
    assert layout_lines(1, fields)[0] == '__LAYOUT__v1 1.0/3 bf motor.position:f'
    assert frame_line(1, fields, seq=7, millis=123456) == '~Ab8HQOIBALbznT8IAAAAAY/t'


def test_core_fields_fit_one_frame_without_their_precision() -> None:
    telemetry = CoreTelemetry(lambda: None)
    code = telemetry.plan(['wheels.linear_speed:4', 'wheels.locked',
                          'p2t.delta_2.angle_left:3', 'p2_ms', 'wheels.locked'])
    assert code == 'core.telemetry(core.millis, wheels.linear_speed, wheels.locked, p2t.delta_2.angle_left, p2_ms, 0)\n'


def test_core_fields_beyond_a_frame_split_into_frames_that_each_start_with_millis() -> None:
    telemetry = CoreTelemetry(lambda: None)
    code = telemetry.plan([f'module.field_{index}:2' for index in range(100)])
    lines = code.splitlines()
    assert len(lines) == 3
    assert all(line.startswith('core.telemetry(core.millis, ') and line.endswith(', 0)') for line in lines)
    assert [len(frame) for frame in telemetry.frames] == [45, 45, 13]  # 4 bytes each, at most 180 bytes


def test_core_fields_must_be_properties_or_variables() -> None:
    telemetry = CoreTelemetry(lambda: None)
    with pytest.raises(ValueError):
        telemetry.plan(['wheels.speed * 2'])
    with pytest.raises(ValueError):
        telemetry.plan(['wheels.speed:x'])


def test_a_frame_becomes_the_text_line_core_output_would_print() -> None:
    telemetry = CoreTelemetry(lambda: None)
    telemetry.plan(['wheels.linear_speed:4', 'wheels.locked', 'p2t.delta_2.state', 'wheels.distance:1',
                    'wheels.locked', 'heading'])
    fields = [('core.millis', 40914), ('wheels.linear_speed', -0.25), ('wheels.locked', True),
              ('p2t.delta_2.state', 3), ('wheels.distance', 12.34), ('heading', 1.75)]
    for line in layout_lines(1, fields):
        assert telemetry.translate(line) is None
    assert telemetry.translate(frame_line(1, fields, millis=40915)) == \
        'core 40914 -0.2500 true 3 12.3 true 2'  # a float without precision prints like "%.0f", as in the text line


def test_the_frames_of_one_step_are_reassembled_by_millis() -> None:
    telemetry = CoreTelemetry(lambda: None)
    telemetry.plan([f'm.f{index}:1' for index in range(50)])
    first, second = telemetry.frames
    lines = layout_lines(1, [(name, 0) for name in first]) + layout_lines(2, [(name, 0) for name in second])
    for line in lines:
        telemetry.translate(line)

    def step(millis: int) -> tuple[str, str]:
        values = {f'm.f{index}': index + millis for index in range(50)}  # ints stand in for the fields
        values['core.millis'] = millis
        return (frame_line(1, [(name, values[name]) for name in first], millis=millis),
                frame_line(2, [(name, values[name]) for name in second], millis=millis))

    a, b = step(100)
    assert telemetry.translate(a) is None
    assert telemetry.translate(b) == 'core 100 ' + ' '.join(str(index + 100) for index in range(50))
    lost, _ = step(110)
    assert telemetry.translate(lost) is None  # its second frame never arrives
    a, b = step(120)
    assert telemetry.translate(a) is None
    assert telemetry.translate(b).startswith('core 120 120 121 ')
    assert telemetry.incomplete_steps == 1


def test_missing_layouts_are_requested_at_most_every_two_seconds() -> None:
    requests: list[float] = []
    clock = Clock()
    telemetry = CoreTelemetry(lambda: requests.append(clock.now), clock=clock)
    telemetry.plan(['wheels.locked'])
    fields = [('core.millis', 5), ('wheels.locked', False)]
    for now in (0.0, 0.5, 1.0, 1.99, 2.0, 3.0, 4.1):
        clock.now = now
        assert telemetry.translate(frame_line(1, fields)) is None
    assert requests == [0.0, 2.0, 4.1]
    assert telemetry.frames_without_layout == 7
    for line in layout_lines(1, fields):
        telemetry.translate(line)
    assert telemetry.translate(frame_line(1, fields)) == 'core 5 false'
    assert requests == [0.0, 2.0, 4.1]


def test_a_frame_of_other_fields_is_dropped(rosys_integration: None) -> None:
    telemetry = CoreTelemetry(lambda: None)
    telemetry.plan(['wheels.locked'])
    fields = [('core.millis', 5), ('wheels.speed', 0.5)]  # an outdated startup on the core
    for line in layout_lines(1, fields):
        telemetry.translate(line)
    assert telemetry.translate(frame_line(1, fields)) is None
    assert telemetry.unexpected_frames == 1


def test_malformed_telemetry_lines_are_dropped() -> None:
    telemetry = CoreTelemetry(lambda: None)
    telemetry.plan(['wheels.locked'])
    good = frame_line(1, [('core.millis', 5), ('wheels.locked', False)])
    assert telemetry.translate(good[:-4] + 'AAA=') is None
    assert telemetry.translate('~not base64') is None
    assert telemetry.malformed_lines == 2


class _Lines(Communication):
    """A communication that hands out queued lines with checksums and records what is sent."""

    def __init__(self) -> None:
        super().__init__()
        self.incoming: list[str] = []
        self.sent: list[str] = []

    @classmethod
    def is_possible(cls) -> bool:
        return True

    async def send(self, msg: str) -> None:
        self.sent.append(msg)

    async def read(self) -> str | None:
        return augment(self.incoming.pop(0)) if self.incoming else None


class _Module(ModuleHardware):
    def __init__(self, robot_brain: RobotBrain) -> None:
        super().__init__(robot_brain=robot_brain, lizard_code='m = Module()',
                         core_message_fields=['m.speed:2', 'm.flag', 'count'])
        self.received: list[list[str]] = []

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.received.append([words.pop(0) for _ in range(3)])


async def test_the_robot_brain_turns_core_frames_into_core_messages(rosys_integration: None) -> None:
    communication = _Lines()
    brain = RobotBrain(communication, enable_esp_on_startup=False, telemetry_frames=True)
    module = _Module(brain)
    robot = RobotHardware([module], brain, rdyp_pin=None, en3_pin=None)
    assert 'core.output' not in brain.lizard_code
    assert brain.lizard_code.endswith('core.telemetry(core.millis, m.speed, m.flag, count, 0)\n')
    millis: list[float] = []
    brain.CORE_MESSAGE_RECEIVED.subscribe(millis.append)

    def fields(step: int) -> list[tuple[str, object]]:
        return [('core.millis', 1000 + 10 * step), ('m.speed', 0.5 * step), ('m.flag', step % 2 == 1), ('count', step)]

    communication.incoming = [frame_line(1, fields(0)), 'bus[2]: hello', frame_line(1, fields(1))]
    assert await brain.read_lines() == []  # nothing counts before the first core message
    await asyncio.sleep(0)
    assert communication.sent == [augment('core.telemetry_info()')]  # once, not for the second frame
    assert millis == []

    communication.incoming = [*layout_lines(1, fields(2)), frame_line(1, fields(2))]
    await robot.update()
    assert millis == [1020.0]
    assert not brain.is_ready  # the first core message only sets the clock offset
    communication.incoming = [frame_line(1, fields(3))]
    await robot.update()
    assert brain.is_ready
    assert module.received == [['1.50', 'true', '3']]

    communication.incoming = [frame_line(1, fields(4)), 'core 9 9 9 9']  # a text line still counts
    await robot.update()
    assert module.received[1:] == [['2.00', 'false', '4'], ['9', '9', '9']]


def test_the_text_path_is_the_default(rosys_integration: None) -> None:
    brain = RobotBrain(_Lines(), enable_esp_on_startup=False)
    RobotHardware([_Module(brain)], brain, rdyp_pin=None, en3_pin=None)
    assert brain.core_telemetry is None
    assert brain.lizard_code.endswith('core.output("core.millis m.speed:2 m.flag count")\n')
