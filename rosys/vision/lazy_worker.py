import asyncio
import uuid
from typing import Awaitable, Callable, Generic, ParamSpec, TypeVar

_T = TypeVar('_T')
_P = ParamSpec('_P')


class LazyWorker(Generic[_P, _T]):

    def __init__(self, func: Callable[_P, Awaitable[_T]]) -> None:
        self.func = func
        self.waiting: uuid.UUID | None = None
        self.is_free = asyncio.Event()
        self.is_free.set()

    async def run(self, *args: _P.args, **kwargs: _P.kwargs) -> _T | None:
        id_ = uuid.uuid4()
        self.waiting = id_
        while True:
            if self.waiting != id_:
                return None
            try:
                await asyncio.wait_for(self.is_free.wait(), timeout=0.1)
            except asyncio.TimeoutError:
                continue
            if self.waiting == id_:
                break

        self.is_free.clear()
        self.waiting = None
        result = await self.func(*args, **kwargs)
        self.is_free.set()
        return result
