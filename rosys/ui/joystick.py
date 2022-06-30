from typing import Literal

from nicegui.elements.joystick import Joystick as NiceGuiJoystick

from ..actors import Steerer


class Joystick(NiceGuiJoystick):

    def __init__(self, steerer: Steerer, **options) -> None:
        self.steerer = steerer
        super().__init__(on_start=self.handle_start, on_move=self.handle_move, on_end=self.handle_end, **options)

    def handle_start(self, _) -> Literal[False]:
        self.steerer.start()
        return False

    def handle_move(self, msg) -> Literal[False]:
        self.steerer.update(msg.data.vector.x, msg.data.vector.y)
        return False

    def handle_end(self, _) -> Literal[False]:
        self.steerer.stop()
        return False
