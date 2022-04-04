from functools import wraps
from typing import Optional

from line_profiler import LineProfiler as PyUtilsLineProfiler


class LineProfiler:

    def __init__(self):
        self.functions: list[list] = []
        self.line_profiler: Optional[PyUtilsLineProfiler] = None

    def __call__(self, func):
        index = len(self.functions)

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
