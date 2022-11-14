from functools import wraps

from nicegui import ui

from .. import rosys


class Track:

    def __init__(self) -> None:
        self.stack: list[str] = []
        self._labels: list[ui.label] = []
        rosys.on_repeat(self.update, 0.5)

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

    def update(self) -> None:
        for label in self._labels:
            label.text = ' → '.join(self.stack)

    def ui(self) -> ui.label:
        self._labels.append(ui.label())
        return self._labels[-1]

    def reset(self) -> None:
        self.stack.clear()
        self.update()


track = Track()
