import asyncio
from collections.abc import Coroutine
from typing import Any, TypeVar

_T = TypeVar('_T')


class LazyWorker:

    def __init__(self) -> None:
        self._condition = asyncio.Event()
        self._is_free = True

    async def run(self, coro: Coroutine[Any, None, _T]) -> _T | None:
        """Run the coroutine and return the result.

        If the worker is busy, the coroutine is put into a queue of size 1.
        All other coroutines are discarded.

        :return: result of the coroutine or ``None`` if the coroutine was discarded
        """
        self._notify()

        if not self._is_free:
            await self._condition.wait()

        if not self._is_free:
            coro.close()
            return None

        self._is_free = False
        try:
            return await coro
        finally:
            self._is_free = True
            self._notify()

    def _notify(self) -> None:
        self._condition.set()
        self._condition.clear()
