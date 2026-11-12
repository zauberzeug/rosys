from nicegui import ui

from .steerer import Steerer


class Joystick(ui.joystick):
    """The Joystick UI element allows controlling a given steerer via touch events."""

    def __init__(self, steerer: Steerer, **options) -> None:
        self.steerer = steerer
        super().__init__(on_start=self.handle_start, on_move=self.handle_move, on_end=self.handle_end, **options)
        self.classes('[&_.back]:opacity-50')  # NOTE: nipplejs 1 renders it opaque, which hides a knob of the same color

    def handle_start(self, _) -> None:
        self.steerer.start()

    def handle_move(self, msg) -> None:
        self.steerer.update(msg.x, msg.y)

    def handle_end(self, _) -> None:
        self.steerer.stop()
