from nicegui.ui import Ui
from rosys.world.point import Point
from rosys.actors.steerer import Steerer


class KeyboardControl():

    # these will be set by rosys.ui.configure
    steerer: Steerer = None
    ui: Ui = None

    def __init__(self, *, default_speed=2) -> None:
        self.ui.keyboard(self.handle_keys)
        self.direction = Point(x=0, y=0)
        self.speed = default_speed

    def adapt_direction(self, e, modifier):
        if e.key.left:
            self.direction.y = - modifier
        elif e.key.right:
            self.direction.y = + modifier
        elif e.key.up:
            self.direction.x = + modifier
        elif e.key.down:
            self.direction.x = - modifier

    def adjust_speed(self, modifier):
        if self.direction.y < 0:
            self.direction.y = - modifier/2
        elif self.direction.y > 0:
            self.direction.y = + modifier/2

        if self.direction.x > 0:
            self.direction.x = + modifier
        elif self.direction.x < 0:
            self.direction.x = - modifier

    def handle_keys(self, e):
        if e.key.repeat:
            return

        if e.action.keydown and e.key.numberkey:
            self.speed = int(e.key.numberkey)
            self.adjust_speed(self.speed)
            self.steerer.update(self.direction.y, self.direction.x)

        if e.key.shift and e.action.keyup:
            self.direction.x = self.direction.y = 0
            self.steerer.stop()

        elif e.modifiers.shiftkey and e.key.is_cursorkey:
            if e.action.keydown:
                if self.direction.x == 0 and self.direction.y == 0:
                    self.steerer.start()
                self.adapt_direction(e, self.speed)
            elif e.action.keyup:
                self.adapt_direction(e, 0)

            self.steerer.update(self.direction.y, self.direction.x)
            if self.direction.x == 0 and self.direction.y == 0:
                self.steerer.stop()
