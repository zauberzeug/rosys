from nicegui.ui import Ui
from rosys.actors.esp import Esp
from rosys.actors.serial_esp import SerialEsp
from rosys import event


class LizardSerialDebug():
    ui: Ui = None  # will be set by rosys.ui.configure
    lizard: Esp = None  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        ic(type(self.lizard))
        if type(self.lizard) is not SerialEsp:
            return
        self.ui.switch('Lizard', value=self.lizard.is_open(), on_change=self.change)

    async def change(self, status):
        if status.value:
            self.lizard.connect()
            await event.call(event.Id.NEW_NOTIFICATION, 'connected to Lizard')
        else:
            await event.call(event.Id.PAUSE_AUTOMATIONS, 'lizard is deactivated')
            self.lizard.disconnect()
            await event.call(event.Id.NEW_NOTIFICATION, 'disconnected from to Lizard')
