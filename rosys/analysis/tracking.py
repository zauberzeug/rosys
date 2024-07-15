import asyncio
from collections import defaultdict
from functools import wraps
from typing import Callable, ParamSpec, TypeVar

from nicegui import ui

_T = TypeVar('_T')
_P = ParamSpec('_P')


class Track:

    def __init__(self) -> None:
        self.stacks: defaultdict[int, list[str]] = defaultdict(list)

    def __call__(self, f: Callable[_P, _T]) -> Callable[_P, _T]:
        @wraps(f)
        async def wrap(*args, **kwargs):
            task_id = id(asyncio.current_task())
            stack = self.stacks[task_id]
            try:
                stack.append(f.__name__)
                return await f(*args, **kwargs)
            finally:
                assert stack
                assert stack[-1] == f.__name__
                stack.pop()
                if not stack:
                    del self.stacks[task_id]
        return wrap

    def ui(self) -> ui.markdown:
        markdown = ui.markdown()
        ui.timer(0.1, lambda: markdown.set_content('\n'.join(' → '.join(stack).replace('_', '\_')
                                                             for stack in self.stacks.values())))
        return markdown


track = Track()
