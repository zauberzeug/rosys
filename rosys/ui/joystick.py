from rosys.actors.steerer import Steerer
from nicegui.elements.joystick import Joystick as NiceGuiJoystick


class Joystick(NiceGuiJoystick):

    def __init__(self, *, steerer: Steerer, **options):
        self.steerer = steerer

        super().__init__(on_start=self.handle_start, on_move=self.handle_move, on_end=self.handle_end, **options)

    def handle_start(self, _):
        self.steerer.start()
        return False

    def handle_move(self, msg):
        self.steerer.update(msg.data.vector.x, msg.data.vector.y)
        return False

    def handle_end(self, _):
        self.steerer.stop()
        return False
