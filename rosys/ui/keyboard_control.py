from rosys.world.point import Point
from rosys.actors.steerer import Steerer


class KeyboardControl():

    def __init__(self, ui, steerer: Steerer) -> None:
        self.steerer = steerer
        self.ui = ui
        self.ui.keyboard(self.handle_keys)
        self.direction = Point(x=0, y=0)
        self.speed = 2

    def adapt_direction(self, e, modifier):
        if e.key.left:
            self.direction.y = - modifier/2
        elif e.key.right:
            self.direction.y = + modifier/2
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

        if e.key.shift:
            if e.action.keydown:
                self.steerer.start()
            else:
                self.direction.x = self.direction.y = 0
                self.steerer.stop()

        elif e.modifiers.shiftkey and e.key.is_cursorkey:
            if e.action.keydown:
                self.adapt_direction(e, self.speed)
            elif e.action.keyup:
                self.adapt_direction(e, 0)

            self.steerer.update(self.direction.y, self.direction.x)
