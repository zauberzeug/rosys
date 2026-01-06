from typing import Literal

from .module import ModuleHardware
from .robot_brain import RobotBrain


class BluetoothHardware(ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, *, name: str = 'robot', pin_code: int | None | Literal['default'] = 'default') -> None:
        """The bluetooth module represents a Bluetooth Low Energy connection for sending messages with optional PIN-based pairing.

        Check https://lizard.dev/module_reference/#bluetooth for more information.

        :param robot_brain: The robot brain instance to communicate with
        :param name: The device name advertised via Bluetooth (default: 'robot')
        :param pin_code: 6-digit PIN (000000-999999) for pairing, ``None`` to deactivate PIN, or "default" for the secret default PIN.
        """
        self.name = name
        lizard_code = f'bluetooth = Bluetooth("{name}")'
        if pin_code != 'default':
            lizard_code += f'\n{self._create_pin_command(pin_code)}'
        super().__init__(robot_brain, lizard_code=lizard_code)

    async def send(self, message: str) -> None:
        """Send a message via Bluetooth notification.

        :param message: The message to send. The needed quotation marks will be added automatically.
        """
        await self.robot_brain.send(f'bluetooth.send("{message}")')

    async def set_pin_code(self, pin_code: int | None | Literal['default']) -> None:
        """Set a PIN code for pairing.

        :param pin_code: 6-digit PIN (000000-999999) for pairing, ``None`` to deactivate PIN, or "default" for the secret default PIN.
        """
        await self.robot_brain.send(self._create_pin_command(pin_code))

    async def reset_bonds(self) -> None:
        """Forget all previously paired devices."""
        await self.robot_brain.send('bluetooth.reset_bonds()')

    @staticmethod
    def _create_pin_command(pin_code: int | None | Literal['default']) -> str:
        if pin_code is None:
            return 'bluetooth.deactivate_pin()'
        if pin_code == 'default':
            return 'bluetooth.reset_pin()'
        if 0 <= pin_code <= 999999:
            return f'bluetooth.set_pin({pin_code:06d})'
        raise ValueError(f'Invalid pin code: {pin_code}')
