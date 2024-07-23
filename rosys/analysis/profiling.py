from functools import wraps

from line_profiler import LineProfiler as PyUtilsLineProfiler


class LineProfiler:

    def __init__(self) -> None:
        self.functions: list[list] = []
        self.line_profiler: PyUtilsLineProfiler | None = None

    def __call__(self, func):
        index = len(self.functions)

        @wraps(func)
        def wrap(*args, **kw):
            return self.functions[index][1](*args, **kw)

        self.functions.append([func, func])
        return wrap

    def start(self) -> None:
        self.line_profiler = PyUtilsLineProfiler()
        for f in self.functions:
            f[1] = self.line_profiler(f[0])

    def stop(self, *, print_stats: bool = True) -> None:
        for f in self.functions:
            f[1] = f[0]
        if print_stats:
            self.print()

    def print(self) -> None:
        if self.line_profiler:
            self.line_profiler.print_stats()

    def reset(self) -> None:
        self.stop(print_stats=False)
        self.start()


profile = LineProfiler()
