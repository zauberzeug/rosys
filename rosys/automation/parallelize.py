from typing import Coroutine


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
        while not all(completed):
            for i, send in enumerate(sends):
                if completed[i]:
                    continue
                try:
                    signal = send(messages[i])
                except StopIteration:
                    if self.return_when_first_completed:
                        return
                    completed[i] = True
                    continue
                else:
                    sends[i] = iter_sends[i]
                try:
                    messages[i] = yield signal
                except GeneratorExit:
                    raise
                except BaseException as e:
                    sends[i], messages[i] = iter_throws[i], e
