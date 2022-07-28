import rosys
from nicegui import ui

from ...automation import Automator
from ...hardware.communication import SerialCommunication


class LizardSerialDebug:

    def __init__(self, communication: SerialCommunication, automator: Automator) -> None:
        self.communication = communication
        self.automator = automator

        ui.switch('Lizard', value=self.communication.serial.isOpen(), on_change=self.toggle_communication)
        self.input = ui.input(on_change=self.submit_message)

    async def submit_message(self) -> None:
        await self.communication.send(self.input.value)
        self.input.value = ''

    def toggle_communication(self, status) -> None:
        assert isinstance(self.communication, SerialCommunication)
        if status.value:
            self.communication.connect()
            rosys.notify('connected to Lizard')
        else:
            self.automator.pause('communication is deactivated')
            self.communication.disconnect()
            rosys.notify('disconnected from Lizard')
