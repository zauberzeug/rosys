from functools import wraps

from nicegui import ui


class Track:

    def __init__(self) -> None:
        self.stack: list[str] = []

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
        label = ui.label()
        ui.timer(0.5, lambda: label.set_text(' → '.join(self.stack)))
        return label

    def reset(self) -> None:
        self.stack.clear()


track = Track()
