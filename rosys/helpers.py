import inspect
import time
from contextlib import contextmanager
from typing import Any, Awaitable, Callable, Generator

import numpy as np


async def invoke(handler: Callable, *args: Any) -> Any:
    result = handler(*args)
    if isinstance(result, Awaitable):
        result = await result
    return result


def measure(*, reset: bool = False, ms: bool = False) -> None:
    global t
    if 't' in globals() and not reset:
        dt = time.perf_counter() - t
        line = inspect.stack()[1][0].f_lineno
        output = f'{dt * 1000:7.3f} ms' if ms else f'{dt:7.3f} s'
        print(f'{inspect.stack()[1].filename}:{line}', output, flush=True)
    if reset:
        print('------------', flush=True)
    t = time.perf_counter()


def angle(yaw0: float, yaw1: float) -> float:
    return eliminate_2pi(yaw1 - yaw0)


def eliminate_pi(angle: float) -> float:
    return (angle + np.pi / 2) % np.pi - np.pi / 2


def eliminate_2pi(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def ramp(x: float, in_min: float, in_max: float, out_min: float, out_max: float, clip: bool = False) -> float:
    if clip and x < in_min:
        return out_min
    if clip and x > in_max:
        return out_max
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class ModificationContext:

    @contextmanager
    def set(self, **kwargs) -> Generator[None, None, None]:
        backup = {key: getattr(self, key) for key in kwargs.keys()}
        for key, value in kwargs.items():
            setattr(self, key, value)
        try:
            yield
        finally:
            for key, value in backup.items():
                setattr(self, key, value)
