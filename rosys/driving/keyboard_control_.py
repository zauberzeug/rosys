import logging

import numpy as np
from nicegui import ui
from nicegui.events import KeyEventArguments

from ..event import Event
from ..geometry import Point
from .steerer import State, Steerer


class KeyboardControl:
    """The KeyboardControl UI element allows controlling a given steerer via keyboard events.

    Hold shift while pressing an arrow key to steer the robot.
    You can change the speed with the number keys 1 to 9 and the initial speed via the `default_speed` argument.
    """

    def __init__(self,
                 steerer: Steerer, *,
                 default_speed: float = 2.0,
                 connection_timeout: float = 1.0,
                 check_connection_interval: float = 1.0) -> None:
        self.CONNECTION_INTERRUPTED = Event[[]]()
        """the keyboard control has lost connection to the browser."""

        self.steerer = steerer
        self.log = logging.getLogger('rosys.ui.keyboard_control')
        ui.keyboard(on_key=self.handle_keys, repeating=False)
        self.direction = Point(x=0, y=0)
        self.speed = default_speed
        self.connection_timeout = connection_timeout
        ui.timer(check_connection_interval, self._check_connection)

    def handle_keys(self, e: KeyEventArguments) -> None:
        self.log.debug('%s -> %s %s', e.key.name, e.action, e.modifiers)

        # change speed via number key
        if e.action.keydown and e.key.number:
            self.speed = e.key.number
            self.direction.y = self.speed * np.sign(self.direction.y)
            self.direction.x = self.speed * np.sign(self.direction.x)
            self.steerer.update(self.direction.y, self.direction.x)

        # update individual speed components via arrow keys
        if e.modifiers.shift and e.key.is_cursorkey:
            if e.action.keydown and self.direction.x == 0 and self.direction.y == 0:
                self.steerer.start()

            new_speed = self.speed if e.action.keydown else 0
            if e.key.arrow_left:
                self.direction.y = -new_speed
            elif e.key.arrow_right:
                self.direction.y = +new_speed
            elif e.key.arrow_up:
                self.direction.x = +new_speed
            elif e.key.arrow_down:
                self.direction.x = -new_speed
            self.steerer.update(self.direction.y, self.direction.x)

            if e.action.keyup and self.direction.x == 0 and self.direction.y == 0:
                self.steerer.stop()

        # stop when releasing shift key
        if e.key.shift and e.action.keyup:
            self.direction.y = 0
            self.direction.x = 0
            self.steerer.stop()

    async def _check_connection(self) -> None:
        if self.steerer.state != State.STEERING:
            return
        try:
            await ui.run_javascript('Date()', timeout=self.connection_timeout)
        except TimeoutError:
            self.log.warning('lost connection to browser')
            self.CONNECTION_INTERRUPTED.emit()
            self.steerer.stop()
