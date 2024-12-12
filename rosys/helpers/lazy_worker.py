import asyncio
from collections.abc import Coroutine
from typing import Any, TypeVar

_T = TypeVar('_T')


class LazyWorker:

    def __init__(self) -> None:
        self._condition = asyncio.Event()
        self._is_free = True

    async def run(self, coro: Coroutine[Any, None, _T]) -> _T | None:
        '''Run the coroutine and return the result.

        If the worker is busy, the coroutine is put in a lifo queue with a size of 1.
        All other coroutines are discarded.

        :return: the result of the coroutine or None if the coroutine was discarded.
        '''
        self._notify()

        if not self._is_free:
            await self._condition.wait()

        if not self._is_free:
            coro.close()
            return None

        self._is_free = False
        try:
            result = await coro
        finally:
            self._is_free = True
            self._notify()

        return result

    def _notify(self) -> None:
        self._condition.set()
        self._condition.clear()
