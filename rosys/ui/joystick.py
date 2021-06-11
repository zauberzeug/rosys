from nicegui.elements.joystick import Joystick as NiceGuiJoystick


class Joystick(NiceGuiJoystick):

    def __init__(self, *, on_drive=None, **options):

        self.orientation = None
        self.on_drive = on_drive

        super().__init__(on_start=self.handle_start, on_move=self.handle_move, on_end=self.handle_end, **options)

    def handle_start(self, _):

        self.orientation = None

    def handle_move(self, msg):

        if self.orientation is None and msg.data.distance > 10:
            self.orientation = -1 if msg.data.direction.angle == 'down' else 1

        if self.orientation is not None and self.on_drive is not None:
            self.on_drive(msg.data.vector.y, -msg.data.vector.x * self.orientation)

    def handle_end(self, _):

        if self.on_drive is not None:
            self.on_drive(0, 0)
