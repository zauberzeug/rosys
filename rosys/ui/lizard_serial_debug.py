from nicegui.ui import Ui
from .. import event
from ..communication import Communication, SerialCommunication


class LizardSerialDebug:
    ui: Ui = None  # will be set by rosys.ui.configure
    communication: Communication = None  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        if not isinstance(self.communication, SerialCommunication):
            return
        self.ui.switch('Lizard', value=self.communication.serial.isOpen(), on_change=self.toggle_communication)
        self.input = self.ui.input(on_change=self.submit_message)

    async def submit_message(self):
        await self.communication.send_async(self.input.value)
        self.input.value = ''
        await self.input.view.update()

    async def toggle_communication(self, status):
        assert isinstance(self.communication, SerialCommunication)
        if status.value:
            self.communication.connect()
            await event.call(event.Id.NEW_NOTIFICATION, 'connected to Lizard')
        else:
            await event.call(event.Id.PAUSE_AUTOMATION, 'communication is deactivated')
            self.communication.disconnect()
            await event.call(event.Id.NEW_NOTIFICATION, 'disconnected from Lizard')
