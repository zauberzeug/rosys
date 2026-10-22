from collections import deque
from collections.abc import AsyncGenerator

import pytest

from rosys.hardware import BluetoothHardware, RobotBrain, RobotHardware
from rosys.hardware.communication import Communication
from rosys.hardware.robot_brain import augment, check
from rosys.testing import forward


class CommunicationSimulation(Communication):

    def __init__(self) -> None:
        super().__init__()
        self.incoming: deque[str] = deque()
        self.sent: list[str] = []

    @classmethod
    def is_possible(cls) -> bool:
        return True

    async def send(self, msg: str) -> None:
        self.sent.append(msg)

    async def read(self) -> str | None:
        return augment(self.incoming.popleft()) if self.incoming else None


@pytest.fixture
async def bluetooth(rosys_integration: None) -> AsyncGenerator[BluetoothHardware, None]:
    communication = CommunicationSimulation()
    robot_brain = RobotBrain(communication, enable_esp_on_startup=False)
    bluetooth = BluetoothHardware(robot_brain, pin_code=None)
    _robot = RobotHardware([bluetooth], robot_brain)  # NOTE: keep alive so its weak update repeater keeps running
    for millis in (100, 200):  # NOTE: the first core message only establishes the clock offset
        communication.incoming.append(f'core {millis}')
        await forward(seconds=1.0)
    assert robot_brain.is_ready
    yield bluetooth


def sent_lines(bluetooth: BluetoothHardware) -> list[str]:
    """Return the ``bluetooth.send`` lines that went over the wire, each with a valid checksum."""
    communication = bluetooth.robot_brain.communication
    assert isinstance(communication, CommunicationSimulation)
    lines = [check(msg) for msg in communication.sent if msg.startswith('bluetooth.send(')]
    assert all(lines), 'every line must carry a valid checksum'
    return lines


@pytest.mark.parametrize('message, line', [
    ('PUT /button/main/play/icon play_arrow',
     'bluetooth.send("PUT /button/main/play/icon play_arrow")'),
    ('POST /notification Enabled WiFi network "Field WiFi" from C:\\wifi',
     'bluetooth.send("POST /notification Enabled WiFi network \\"Field WiFi\\" from C:\\\\wifi")'),
    ('POST /notification Driving failed:\r\nobstacle ahead\nstopped\n',
     'bluetooth.send("POST /notification Driving failed: obstacle ahead stopped")'),
    ('POST /notification \\"',
     'bluetooth.send("POST /notification \\\\\\"")'),
], ids=['plain', 'quotes_and_backslashes', 'line_breaks', 'escaped_quote'])
async def test_a_message_is_escaped_for_lizard(bluetooth: BluetoothHardware, message: str, line: str):
    await bluetooth.send(message)
    assert sent_lines(bluetooth) == [line]
