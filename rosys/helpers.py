import asyncio
import inspect
import sys
import time
from contextlib import contextmanager

import numpy as np


def measure(*, reset: bool = False, ms: bool = False):
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


is_test: bool = 'pytest' in sys.modules


async def sleep(seconds: float):
    if is_test:
        from rosys.test.helper import global_runtime
        sleep_end_time = global_runtime.world.time + seconds
        while global_runtime.world.time <= sleep_end_time:
            await asyncio.sleep(0)
    else:
        count = int(np.ceil(seconds))
        if count > 0:
            for _ in range(count):
                await asyncio.sleep(seconds / count)
        else:
            await asyncio.sleep(0)


class ModificationContext:

    @contextmanager
    def set(self, **kwargs):
        backup = {key: getattr(self, key) for key in kwargs.keys()}
        for key, value in kwargs.items():
            setattr(self, key, value)
        try:
            yield
        finally:
            for key, value in backup.items():
                setattr(self, key, value)
