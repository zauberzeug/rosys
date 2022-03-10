from line_profiler import LineProfiler as PyUtilsLineProfiler
from functools import wraps
from typing import Awaitable, Callable, Optional, Union
from inspect import iscoroutinefunction


class LineProfiler:

    def __init__(self):
        self.functions: list[list] = []
        self.line_profiler: Optional[PyUtilsLineProfiler] = None

    def __call__(self, func: Union[Callable, Awaitable]):
        index = len(self.functions)

        if iscoroutinefunction(func):
            @wraps(func)
            async def wrap(*args, **kw):
                return await self.functions[index][1](*args, **kw)

            self.functions.append([func, func])
            return wrap
        else:
            @wraps(func)
            def wrap(*args, **kw):
                return self.functions[index][1](*args, **kw)

            self.functions.append([func, func])
            return wrap

    def start(self):
        self.line_profiler = PyUtilsLineProfiler()
        for f in self.functions:
            f[1] = self.line_profiler(f[0])

    def stop(self, *, print: bool = True):
        for f in self.functions:
            f[1] = f[0]
        if print:
            self.print()

    def print(self):
        if self.line_profiler:
            self.line_profiler.print_stats()

    def reset(self):
        self.stop(print=False)
        self.start()


profile = LineProfiler()
