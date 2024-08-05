import asyncio
from collections.abc import Coroutine


class parallelize:
    """Parallelize multiple coroutines.

    This class allows to combine multiple coroutines into one that can be passed to the
    `automator <https://rosys.io/reference/rosys/automation/#rosys.automation.Automator>`__
    to run them in parallel.
    """

    def __init__(self, *coros: Coroutine, return_when_first_completed: bool = False) -> None:
        self.coros = coros
        self.return_when_first_completed = return_when_first_completed

    def __await__(self):
        coro_iters = [coro.__await__() for coro in self.coros]
        iter_sends = [coro_iter.send for coro_iter in coro_iters]
        iter_throws = [coro_iter.throw for coro_iter in coro_iters]
        sends = iter_sends[:]
        messages = [None for _ in sends]
        completed = [False for _ in sends]
        waiting = [False for _ in sends]
        futures = [None for _ in sends]

        try:
            while not all(completed):
                active_coros = [i for i, is_completed in enumerate(completed) if not is_completed]

                for i in active_coros:
                    if waiting[i]:
                        if futures[i].done():
                            waiting[i] = False
                            futures[i] = None
                        else:
                            continue

                    try:
                        signal = sends[i](messages[i])
                    except StopIteration:
                        completed[i] = True
                        if self.return_when_first_completed:
                            return
                        continue
                    else:
                        sends[i] = iter_sends[i]

                    if isinstance(signal, asyncio.Future | asyncio.Task):
                        waiting[i] = True
                        futures[i] = signal
                    else:
                        try:
                            messages[i] = yield signal
                        except GeneratorExit:
                            raise
                        except BaseException as e:
                            sends[i], messages[i] = iter_throws[i], e

                if all(waiting[i] or completed[i] for i in active_coros):
                    yield  # Allow event loop to process other tasks
        finally:
            # Ensure all coroutines are properly closed
            close_exceptions = []
            for i, is_completed in enumerate(completed):
                if not is_completed:
                    try:
                        coro_iters[i].close()
                    except Exception as e:
                        close_exceptions.append((i, e))

            # If any exceptions occurred during closing, raise a RuntimeError with details
            if close_exceptions:
                error_msg = 'Exceptions occurred while closing coroutines:\n'
                error_msg += '\n'.join(f'Coroutine {i}: {e}' for i, e in close_exceptions)
                raise RuntimeError(error_msg)
