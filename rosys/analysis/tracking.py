import asyncio
import uuid
from collections import defaultdict
from collections.abc import Callable, Coroutine
from functools import wraps
from typing import ParamSpec, TypeVar

from nicegui import ui

_T = TypeVar('_T')
_P = ParamSpec('_P')


class Track:

    def __init__(self) -> None:
        self.stacks: defaultdict[int, dict[int, str]] = defaultdict(dict)

    def __call__(self, f: Callable[_P, Coroutine[None, None, _T]]) -> Callable[_P, Coroutine[None, None, _T]]:
        @wraps(f)
        async def wrap(*args, **kwargs):
            task_id = id(asyncio.current_task())
            stack = self.stacks[task_id]
            uid = uuid.uuid4().int
            stack[uid] = f.__name__
            try:
                return await f(*args, **kwargs)
            finally:
                del stack[uid]
                if not stack:
                    del self.stacks[task_id]
        return wrap

    def ui(self) -> ui.markdown:
        markdown = ui.markdown()
        ui.timer(0.1, lambda: markdown.set_content('\n\n'.join(' → '.join(stack.values()).replace('_', r'\_')
                                                               for stack in self.stacks.values())))
        return markdown


track = Track()
