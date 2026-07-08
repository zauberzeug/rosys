from collections import deque

import pytest
from nicegui import background_tasks

import rosys
from rosys.hardware import RobotBrain, RobotHardware
from rosys.hardware.communication import Communication
from rosys.hardware.robot_brain import augment
from rosys.testing import forward

MISMATCH_MESSAGE = 'Lizard startup code is outdated'


class CommunicationSimulation(Communication):

    def __init__(self) -> None:
        super().__init__()
        self.incoming: deque[str] = deque()
        self.sent: list[str] = []
        self.startup_checksum = ''

    @classmethod
    def is_possible(cls) -> bool:
        return True

    async def send(self, msg: str) -> None:
        line = msg.rsplit('@', 1)[0]
        self.sent.append(line)
        if line == 'core.startup_checksum()':
            self.incoming.append(f'checksum: {self.startup_checksum}')

    async def read(self) -> str | None:
        return augment(self.incoming.popleft()) if self.incoming else None


def matching_checksum(robot_brain: RobotBrain) -> str:
    return f'{sum(ord(c) for c in robot_brain.lizard_code) % 0x10000:04x}'


async def connect(communication: CommunicationSimulation) -> None:
    for millis in (100, 200):  # NOTE: the first core message only establishes the clock offset
        communication.incoming.append(f'core {millis}')
        await forward(seconds=1.0)
    await forward(seconds=3.0)  # NOTE: let the checksum request and response complete


@pytest.fixture
async def robot_brain(rosys_integration: None) -> RobotBrain:
    robot_brain = RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False)
    RobotHardware([], robot_brain)
    return robot_brain


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
