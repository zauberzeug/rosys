import asyncio
from collections.abc import Coroutine

from .automation import AutomationStopped, _uninterruptible_depth


class parallelize:
    """Parallelize multiple coroutines.

    This class allows to combine multiple coroutines into one that can be passed to the
    `automator <https://rosys.io/reference/rosys/automation/#rosys.automation.Automator>`__
    to run them in parallel.

    Note that ``parallelize`` will be uninterruptible if one of its coroutines is marked with ``@rosys.automation.uninterruptible``.
    This also holds for ``return_when_first_completed`` and for a failing coroutine: the remaining coroutines are only
    stopped (and a failure re-raised) once none of them is inside an uninterruptible section anymore.
    A section the caller itself is in does not hold the stop back.
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
        stop_when_interruptible = False  # a first-completed stop or a failure, held back by an uninterruptible section
        exception_to_reraise: list[BaseException] = []  # holds the external stop to re-raise once draining is done
        outer_depth = _uninterruptible_depth()  # sections the caller is in do not belong to the coroutines

        def in_uninterruptible_section() -> bool:
            return _uninterruptible_depth() > outer_depth

        def cancel_unfinished(exception: BaseException, *, propagate: bool) -> None:
            """Inject *exception* into every unfinished coroutine so its cleanup (incl. ``await``) runs."""
            nonlocal cancelling, stop_when_interruptible
            cancelling = True
            stop_when_interruptible = False  # a cancellation supersedes a held-back stop
            if propagate and not exception_to_reraise:
                exception_to_reraise.append(exception)
            for i, is_completed in enumerate(completed):
                if not is_completed:
                    sends[i], messages[i], waiting[i] = iter_throws[i], exception, None

        try:
            while not all(completed):
                if stop_when_interruptible and not in_uninterruptible_section():
                    stop_when_interruptible = False
                    cancel_unfinished(AutomationStopped(), propagate=False)
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
                            # stop the remaining coroutines, but let their cleanup run before returning;
                            # an uninterruptible one first runs to the end of its section
                            if in_uninterruptible_section():
                                stop_when_interruptible = True
                            else:
                                cancel_unfinished(AutomationStopped(), propagate=False)
                        continue
                    except BaseException as e:
                        # the coroutine finished by (re-)raising; while draining a cancellation this is expected
                        completed[i] = True
                        if not cancelling:
                            # a genuine failure: propagate it (closing the remaining coroutines in finally),
                            # unless an uninterruptible sibling first has to run to the end of its section
                            if not in_uninterruptible_section():
                                if exception_to_reraise:
                                    # the failure held back for the section came first
                                    raise exception_to_reraise[0] from e
                                raise
                            if not exception_to_reraise:
                                exception_to_reraise.append(e)
                            stop_when_interruptible = True
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
                    pending = [w for w in (waiting[i] for i in active_coros) if w is not None]
                    if pending:
                        try:
                            # suspend until any coroutine can make progress instead of busy-polling the event loop
                            yield from asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED).__await__()
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
