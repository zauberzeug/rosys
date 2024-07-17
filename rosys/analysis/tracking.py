import uuid
from functools import wraps
from typing import Callable, ParamSpec, TypeVar

from nicegui import ui

_T = TypeVar('_T')
_P = ParamSpec('_P')


class Track:

    def __init__(self) -> None:
        self.stack: dict[int, str] = {}

    def __call__(self, f: Callable[_P, _T]) -> Callable[_P, _T]:
        @wraps(f)
        async def wrap(*args, **kwargs):
            uid = uuid.uuid4().int
            self.stack[uid] = f.__name__
            try:
                return await f(*args, **kwargs)
            finally:
                del self.stack[uid]
        return wrap

    def ui(self) -> ui.label:
        label = ui.label()
        ui.timer(0.5, lambda: label.set_text(' → '.join(self.stack.values())))
        return label


track = Track()
