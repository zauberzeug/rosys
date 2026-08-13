from collections import deque
from collections.abc import Generator

import pytest
from nicegui import background_tasks

import rosys
from rosys.hardware import RobotBrain, RobotHardware
from rosys.hardware.communication import Communication
from rosys.hardware.robot_brain import MAX_CONFIGURE_ATTEMPTS, augment, check
from rosys.testing import forward

MISMATCH_MESSAGE = 'Lizard startup code is outdated'
MATCHING = 'matching'  # NOTE: placeholder for the checksum of the actual Lizard code, which is only known in the test


class CommunicationSimulation(Communication):

    def __init__(self) -> None:
        super().__init__()
        self.incoming: deque[str] = deque()
        self.sent: list[str] = []
        self.startup_checksum = ''
        self.startup_checksums: deque[str | None] = deque()  # NOTE: ``None`` simulates a missing response

    @classmethod
    def is_possible(cls) -> bool:
        return True

    async def send(self, msg: str) -> None:
        line = msg.rsplit('@', 1)[0]
        self.sent.append(line)
        if line == 'core.startup_checksum()':
            checksum = self.startup_checksums.popleft() if self.startup_checksums else self.startup_checksum
            if checksum is not None:
                self.startup_checksum = checksum
                self.incoming.append(f'checksum: {checksum}')

    async def read(self) -> str | None:
        return augment(self.incoming.popleft()) if self.incoming else None


def matching_checksum(robot_brain: RobotBrain) -> str:
    startup = ''.join(f'{line}\n' for line in robot_brain.lizard_code.splitlines())
    return f'{sum(startup.encode()) % 0x10000:04x}'


async def connect(communication: CommunicationSimulation) -> None:
    for millis in (100, 200):  # NOTE: on a first connect, the first core message only establishes the clock offset
        communication.incoming.append(f'core {millis}')
        await forward(seconds=1.0)
    await forward(seconds=3.0)  # NOTE: let the checksum request and response complete


@pytest.fixture
def robot_brain(rosys_integration: None) -> Generator[RobotBrain, None, None]:
    robot_brain = RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False)
    _robot = RobotHardware([], robot_brain)  # NOTE: keep alive so its weak update repeater keeps running
    yield robot_brain


async def test_no_warning_when_lizard_code_matches(robot_brain: RobotBrain) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    communication.startup_checksum = matching_checksum(robot_brain)
    await connect(communication)
    assert 'core.startup_checksum()' in communication.sent
    assert not any(MISMATCH_MESSAGE in message for message in notifications)


async def test_warning_when_lizard_code_differs(robot_brain: RobotBrain) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    communication.startup_checksum = 'ffff'
    await connect(communication)
    assert any(MISMATCH_MESSAGE in message for message in notifications)


async def test_check_runs_again_after_configuring(robot_brain: RobotBrain) -> None:
    connect_count = 0

    def count_connect() -> None:
        nonlocal connect_count
        connect_count += 1
    robot_brain.ESP_CONNECTED.subscribe(count_connect)
    communication = robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    communication.startup_checksum = 'ffff'
    await connect(communication)
    assert robot_brain.lizard_firmware.checksums_match is False

    communication.startup_checksum = matching_checksum(robot_brain)
    configure_task = background_tasks.create(robot_brain.configure(), name='configure')
    communication.incoming.append('core 5000')  # NOTE: buffered message arriving after ``core.restart()``
    await forward(seconds=2.0)
    assert configure_task.done()
    assert not robot_brain.is_ready, 'buffered messages should not re-establish the connection'
    assert connect_count == 1

    await connect(communication)
    assert connect_count == 2
    assert robot_brain.lizard_firmware.checksums_match is True


@pytest.mark.parametrize('checksums, reason', [
    (['ffff', MATCHING], None),
    (['ffff'] * MAX_CONFIGURE_ATTEMPTS, 'checksum mismatch (ffff instead of'),
    ([None] * MAX_CONFIGURE_ATTEMPTS, 'no checksum received'),
], ids=['retry_then_match', 'repeated_mismatch', 'no_response'])
async def test_configure_verifies_startup_checksum(robot_brain: RobotBrain,
                                                   checksums: list[str | None], reason: str | None) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    communication.startup_checksum = matching_checksum(robot_brain)
    await connect(communication)
    communication.startup_checksums.extend(matching_checksum(robot_brain) if checksum is MATCHING else checksum
                                           for checksum in checksums)
    task = background_tasks.create(robot_brain.configure(), name='configure')
    await forward(seconds=15.0)
    assert communication.sent.count('!-') == len(checksums)
    assert communication.sent.count('!.') == (0 if reason else 1), 'only a verified script may be persisted'
    assert communication.sent.count('core.restart()') == 1, \
        'restart applies the persisted script -- or restores it into RAM after a failed upload'
    assert task.result() == (reason is None)
    assert any(f'Configuring Lizard failed: {reason}' in message for message in notifications) == (reason is not None)


async def test_startup_checksum_requests_are_serialized(robot_brain: RobotBrain) -> None:
    communication = robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    communication.startup_checksum = matching_checksum(robot_brain)
    await connect(communication)
    requests = communication.sent.count('core.startup_checksum()')
    communication.startup_checksums.append(None)  # NOTE: leave the first request without a response
    first = background_tasks.create(robot_brain.read_startup_checksum(timeout=2.0), name='first')
    second = background_tasks.create(robot_brain.read_startup_checksum(timeout=2.0), name='second')
    await forward(seconds=1.0)
    assert communication.sent.count('core.startup_checksum()') == requests + 1, \
        'a second request must not compete for the shared "checksum:" response slot'
    await forward(seconds=5.0)
    assert first.result() is None
    assert second.result() == matching_checksum(robot_brain), 'the waiting request should still get its own response'


async def test_configure_does_not_deadlock_with_the_startup_check(robot_brain: RobotBrain) -> None:
    communication = robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    communication.startup_checksum = matching_checksum(robot_brain)
    await connect(communication)
    await robot_brain.restart()
    task = background_tasks.create(robot_brain.configure(), name='configure')
    communication.incoming.append('core 5000')  # NOTE: the Core reconnects and triggers _check_lizard_code
    await forward(seconds=10.0)
    assert task.result() is True, 'waiting for the serialized checksum request must not stall a good upload'
    assert communication.sent.count('!.') == 1


async def test_configure_without_a_trailing_newline(robot_brain: RobotBrain) -> None:
    robot_brain.lizard_code = 'core.debug = true'
    communication = robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    communication.startup_checksum = matching_checksum(robot_brain)
    await connect(communication)
    task = background_tasks.create(robot_brain.configure(), name='configure')
    await forward(seconds=2.0)
    assert task.result() is True
    assert communication.sent.count('!.') == 1


async def test_local_checksum_matches_what_lizard_stores(robot_brain: RobotBrain) -> None:
    robot_brain.lizard_code = 'grün'
    robot_brain.lizard_firmware.read_local_checksum()
    # NOTE: Lizard sums the raw bytes of the stored startup script, one newline-terminated line per '!+' command
    # (0x67 + 0x72 + 0xc3 + 0xbc + 0x6e + 0x0a = 0x02d0)
    assert robot_brain.lizard_firmware.local_checksum == '02d0'
    robot_brain.lizard_code = 'grün\n'
    robot_brain.lizard_firmware.read_local_checksum()
    assert robot_brain.lizard_firmware.local_checksum == '02d0', 'a trailing newline must not change the checksum'


@pytest.mark.parametrize('line, checksum', [
    ('hello', '62'),
    ('wheels.speed(1, 2)', '47'),
    ('grün', '04'),  # NOTE: XOR over the UTF-8 bytes, like Lizard, not over code points (which would give '87')
    ('café', '0e'),
    ('ß', '5c'),
    ('日本', '02'),  # NOTE: code points would XOR to 0x2c9 and overflow the two-digit '{:02x}' format
])
def test_augment_and_check(line: str, checksum: str) -> None:
    assert augment(line) == f'{line}@{checksum}'
    assert check(augment(line)) == line


@pytest.mark.parametrize('line', ['foo@zz', 'foo@1z', 'foo@-1', '\ud800@ff'])
def test_check_rejects_corrupted_lines(line: str) -> None:
    assert check(line) == ''


def test_augment_rejects_undecodable_line() -> None:
    with pytest.raises(UnicodeEncodeError):
        augment('\ud800')
