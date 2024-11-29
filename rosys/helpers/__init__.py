import functools
import inspect
import logging
import logging.config
import os
import sys
import time
from collections.abc import Awaitable, Callable, Generator
from contextlib import contextmanager
from typing import Any

import numpy as np
from nicegui import app

from .lazy_worker import LazyWorker

__all__ = [
    'LazyWorker',
    'ModificationContext',
    'PackagePathFilter',
    'angle',
    'eliminate_2pi',
    'eliminate_pi',
    'from_dict',
    'invoke',
    'is_stopping',
    'is_test',
    'measure',
    'ramp',
    'remove_indentation',
]


async def invoke(handler: Callable, *args: Any) -> Any:
    result = handler(*args)
    if isinstance(result, Awaitable):
        result = await result
    return result


def measure(*, reset: bool = False, ms: bool = False) -> None:
    if 't' in globals() and not reset:
        dt = time.perf_counter() - globals()['t']
        line = inspect.stack()[1][0].f_lineno
        output = f'{dt * 1000:7.3f} ms' if ms else f'{dt:7.3f} s'
        print(f'{inspect.stack()[1].filename}:{line}', output, flush=True)
    if reset:
        print('------------', flush=True)
    globals()['t'] = time.perf_counter()


def angle(yaw0: float, yaw1: float) -> float:
    return eliminate_2pi(yaw1 - yaw0)


def eliminate_pi(angle_: float) -> float:
    return (angle_ + np.pi / 2) % np.pi - np.pi / 2


def eliminate_2pi(angle_: float) -> float:
    return (angle_ + np.pi) % (2 * np.pi) - np.pi


def ramp(x: float, in1: float, in2: float, out1: float, out2: float, clip: bool = False) -> float:
    """Map a value x from one range (in1, in2) to another (out1, out2)."""
    if clip:
        if x < in1 < in2 or in2 < in1 < x:
            return out1
        if x < in2 < in1 or in1 < in2 < x:
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
        backup = {key: getattr(self, key) for key in kwargs}
        for key, value in kwargs.items():
            setattr(self, key, value)
        try:
            yield
        finally:
            for key, value in backup.items():
                setattr(self, key, value)


def from_dict(data_class_type, data):
    if data is None:
        return None
    if hasattr(data_class_type, '__dataclass_fields__'):  # It's a DataClass
        field_types = dict(data_class_type.__annotations__.items())
        return data_class_type(**{f: from_dict(field_types[f], data[f]) for f in data})
    if hasattr(data_class_type, '__origin__'):  # It's a generic
        if data_class_type.__origin__ is list:  # It's a List
            return [from_dict(data_class_type.__args__[0], d) for d in data]
        if data_class_type.__origin__ is dict:  # It's a Dict
            return {k: from_dict(data_class_type.__args__[1], v) for k, v in data.items()}
        raise NotImplementedError(f'Unknown generic type: {data_class_type}')
    # It's a regular type
    return data_class_type(data)


def is_stopping() -> bool:
    return app.is_stopping


def is_test() -> bool:
    return 'pytest' in sys.modules


# https://stackoverflow.com/a/52582536/3419103
@functools.lru_cache(maxsize=100)
def find_relative_path(pathname: str) -> str | None:
    for path in PackagePathFilter.ABS_SYS_PATHS:
        if pathname.startswith(path):
            return os.path.relpath(pathname, path)
    return None


class PackagePathFilter(logging.Filter):
    ABS_SYS_PATHS = sorted((os.path.abspath(p) + os.sep for p in sys.path), key=len, reverse=True)

    def filter(self, record: logging.LogRecord) -> bool:
        record.relative_path = find_relative_path(record.pathname)
        return True
