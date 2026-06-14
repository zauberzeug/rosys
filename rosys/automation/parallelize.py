import asyncio
from collections.abc import Coroutine

from .automation import AutomationStopped


class parallelize:
    """Parallelize multiple coroutines.

    This class allows to combine multiple coroutines into one that can be passed to the
    `automator <https://rosys.io/reference/rosys/automation/#rosys.automation.Automator>`__
    to run them in parallel.

    Note that ``parallelize`` will be uninterruptible if one of its coroutines is marked with ``@rosys.automation.uninterruptible``.
    """

    def __init__(self, *coros: Coroutine, return_when_first_completed: bool = False) -> None:
        self.coros = coros
        self.return_when_first_completed = return_when_first_completed

    def __await__(self):
        coro_iters = [coro.__await__() for coro in self.coros]
        iter_sends = [coro_iter.send for coro_iter in coro_iters]
        iter_throws = [coro_iter.throw for coro_iter in coro_iters]
        sends = iter_sends[:]
        messages: list = [None for _ in sends]
        completed = [False for _ in sends]
        waiting: list[asyncio.Future | asyncio.Task | None] = [None for _ in sends]
        cancelling = False  # whether a cancellation is being drained through the unfinished coroutines
        exception_to_reraise: list[BaseException] = []  # holds the external stop to re-raise once draining is done

        def cancel_unfinished(exception: BaseException, *, propagate: bool) -> None:
            """Inject *exception* into every unfinished coroutine so its cleanup (incl. ``await``) runs."""
            nonlocal cancelling
            cancelling = True
            if propagate and not exception_to_reraise:
                exception_to_reraise.append(exception)
            for i, is_completed in enumerate(completed):
                if not is_completed:
                    sends[i], messages[i], waiting[i] = iter_throws[i], exception, None

        try:
            while not all(completed):
                active_coros = [i for i, is_completed in enumerate(completed) if not is_completed]

                for i in active_coros:
                    task = waiting[i]
                    if task is not None:
                        if task.done():
                            waiting[i] = None
                        else:
                            continue

                    try:
                        signal = sends[i](messages[i])
                    except StopIteration:
                        completed[i] = True
                        if self.return_when_first_completed and not cancelling:
                            # stop the remaining coroutines, but let their cleanup run before returning
                            cancel_unfinished(AutomationStopped(), propagate=False)
                        continue
                    except BaseException:
                        # the coroutine finished by (re-)raising; while draining a cancellation this is expected
                        completed[i] = True
                        if not cancelling:
                            raise  # a genuine failure: propagate it (closing the remaining coroutines in finally)
                        continue
                    else:
                        sends[i] = iter_sends[i]

                    if isinstance(signal, asyncio.Future | asyncio.Task):
                        waiting[i] = signal
                    else:
                        try:
                            messages[i] = yield signal
                        except GeneratorExit:
                            raise  # cannot drive async cleanup under GeneratorExit; let finally close the coroutines
                        except BaseException as e:
                            cancel_unfinished(e, propagate=True)

                if all(waiting[i] is not None or completed[i] for i in active_coros):
                    try:
                        yield  # allow the event loop to process other tasks while all coroutines are waiting
                    except GeneratorExit:
                        raise
                    except BaseException as e:
                        cancel_unfinished(e, propagate=True)

            if exception_to_reraise:
                # re-raise the external stop now that every coroutine has cleaned up
                raise exception_to_reraise[0]
        finally:
            # Close any coroutine that never cleaned up (e.g. a GeneratorExit aborted the draining)
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
