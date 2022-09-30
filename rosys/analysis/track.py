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
            try:
                self.stack.append(f.__name__)
                return await f(*args, **kw)
            finally:
                if self.stack and self.stack[-1] == f.__name__:
                    self.stack.pop()
        return wrap

    def ui(self) -> ui.label:
        def update() -> None:
            self._ui.text = ' → '.join(self.stack)
        self._ui = ui.label()
        ui.timer(0.5, update)
        return self._ui

    def reset(self) -> None:
        self.stack.clear()
        if self._ui:
            self._ui.text = ''


track = Track()
