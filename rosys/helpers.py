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


def ramp(x: float, in1: float, in2: float, out1: float, out2: float, clip: bool = False) -> float:
    """Map a value x from one range (in1, in2) to another (out1, out2)."""
    if clip and x < min(in1, in2):
        return out1
    if clip and x > max(in1, in2):
        return out2
    return (x - in1) * (out2 - out1) / (in2 - in1) + out1


def remove_indentation(text: str) -> str:
    """Remove indentation from a multi-line string based on the indentation of the first line."""
    lines = text.splitlines()
    while lines and not lines[0].strip():
        lines.pop(0)
    if not lines:
        return ''
    indentation = len(lines[0]) - len(lines[0].lstrip())
    return '\n'.join(line[indentation:] for line in lines)


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
