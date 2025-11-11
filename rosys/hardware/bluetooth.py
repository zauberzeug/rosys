from .module import ModuleHardware
from .robot_brain import RobotBrain


class BluetoothHardware(ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, *, name: str = 'robot', pin_code: str | None = None) -> None:
        """The bluetooth module represents a Bluetooth Low Energy connection for sending messages with optional PIN-based pairing.

        Check https://lizard.dev/module_reference/#bluetooth for more information.

        :param robot_brain: The robot brain instance to communicate with
        :param name: The device name advertised via Bluetooth (default: 'robot')
        :param pin_code: Optional 6-digit PIN code (000000-999999) for pairing. If None, a secret default PIN is used. Use empty string to deactivate PIN.
        """
        self.name = name
        lizard_code = f'bluetooth = Bluetooth("{name}")'
        if pin_code:
            self._validate_pin_code(pin_code)
            lizard_code += f'\nbluetooth.set_pin({pin_code})'
        elif pin_code == '':
            lizard_code += '\nbluetooth.deactivate_pin()'
        super().__init__(robot_brain, lizard_code=lizard_code)

    async def send(self, message: str) -> None:
        """Send a message via Bluetooth notification.

        :param message: The message to send. The needed quotation marks will be added automatically.
        """
        await self.robot_brain.send(f'bluetooth.send("{message}")')

    async def set_pin_code(self, pin_code: str) -> None:
        """Set a PIN code for pairing.

        :param pin_code: A PIN code between 000000 and 999999.
        """
        self._validate_pin_code(pin_code)
        await self.robot_brain.send(f'bluetooth.set_pin({pin_code})')

    async def reset_pin_code(self) -> None:
        """Remove the user PIN code and use the default PIN."""
        await self.robot_brain.send('bluetooth.reset_pin()')

    async def deactivate_pin_code(self) -> None:
        """Disable PIN enforcement."""
        await self.robot_brain.send('bluetooth.deactivate_pin()')

    async def reset_bonds(self) -> None:
        """Forget all previously paired devices."""
        await self.robot_brain.send('bluetooth.reset_bonds()')

    def _validate_pin_code(self, pin_code: str) -> None:
        """Validate the given PIN code.

        :param pin_code: The PIN code to validate.
        :raises ValueError: If the PIN code is not a 6-digit numerical string between 000000 and 999999.
        """
        if len(pin_code) != 6:
            raise ValueError('Pin code must be 6 digits long')
        if not 0 <= int(pin_code) <= 999999:
            raise ValueError('Pin code must be between 000000 and 999999')
