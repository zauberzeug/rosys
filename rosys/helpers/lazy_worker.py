import asyncio
from typing import Any, Coroutine, Generic, TypeVar

_T = TypeVar('_T')


class LazyWorker(Generic[_T]):

    def __init__(self) -> None:
        self.waiting: asyncio.Task | None = None
        self.is_free = asyncio.Event()
        self.is_free.set()

    async def run(self, coro: Coroutine[Any, None, _T]) -> _T | None:
        task: asyncio.Task[_T] = asyncio.create_task(coro)
        self.waiting = task
        while True:
            if self.waiting != task:
                task.cancel()
                return None
            try:
                await asyncio.wait_for(self.is_free.wait(), timeout=0.1)
            except asyncio.TimeoutError:
                continue
            if self.waiting == task:
                break

        self.is_free.clear()
        self.waiting = None
        result = await task
        self.is_free.set()
        return result
