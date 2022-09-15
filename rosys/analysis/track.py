from functools import wraps
from typing import Optional

from nicegui import ui


class Track:

    def __init__(self) -> None:
        self.stack: list[str] = []
        self._ui: Optional[ui.label] = None

    def __call__(self, f):
        @wraps(f)
        async def wrap(*args, **kw):
            self.stack.append(f.__name__)
            result = await f(*args, **kw)
            self.stack.pop()
            return result
        return wrap

    def ui(self) -> ui.label:
        def update() -> None:
            self._ui.text = ' → '.join(self.stack)
        self._ui = ui.label()
        ui.timer(0.5, update)
        return self._ui


track = Track()
