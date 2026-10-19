from rosys.hardware import BluetoothHardware


class RobotBrainStub:

    def __init__(self) -> None:
        self.sent: list[str] = []

    async def send(self, msg: str) -> None:
        self.sent.append(msg)


async def test_quotes_and_backslashes_in_a_message_are_escaped_for_lizard():
    robot_brain = RobotBrainStub()
    bluetooth = BluetoothHardware(robot_brain, pin_code=None)  # type: ignore[arg-type]
    await bluetooth.send('POST /notification Enabled WiFi network "Field WiFi" from C:\\wifi')
    assert robot_brain.sent == [
        'bluetooth.send("POST /notification Enabled WiFi network \\"Field WiFi\\" from C:\\\\wifi")',
    ]


async def test_a_plain_message_is_sent_unchanged():
    robot_brain = RobotBrainStub()
    bluetooth = BluetoothHardware(robot_brain, pin_code=None)  # type: ignore[arg-type]
    await bluetooth.send('PUT /button/main/play/icon play_arrow')
    assert robot_brain.sent == ['bluetooth.send("PUT /button/main/play/icon play_arrow")']
