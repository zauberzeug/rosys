import asyncio
from collections.abc import Coroutine
from typing import Any, TypeVar

_T = TypeVar('_T')


class LazyWorker:

    def __init__(self) -> None:
        self._condition = asyncio.Event()
        self._is_free = True

    async def run(self, coro: Coroutine[Any, None, _T]) -> _T | None:
        self._notify()

        if not self._is_free:
            await self._condition.wait()

        if not self._is_free:
            coro.close()
            return None

        self._is_free = False
        result = await coro
        self._is_free = True
        self._notify()
        return result

    def _notify(self) -> None:
        self._condition.set()
        self._condition.clear()
