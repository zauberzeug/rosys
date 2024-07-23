import logging
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from ...rosys import on_repeat


@dataclass(slots=True, kw_only=True)
class Measurement:
    time: str
    duration: float
    name: str
    details: str


color_pattern = re.compile(r'(?:\x1B[@-_]|[\x80-\x9F])[0-?]*[ -/]*[@-~]')
warning_pattern = re.compile(r'(.*) \[WARNING\].*Executing(.*)took (.*) seconds')
task_pattern = re.compile(r".*name=['\"](.*)['\"] coro=<(.*)> .*")
coro_pattern = re.compile(r'(.*) (running at .*)')


class AsyncioMonitor:

    def __init__(self, log_filepath: Path = Path('~/.rosys/debug.log')) -> None:
        self.log = logging.getLogger('rosys.asyncio_monitor')

        self.log_filepath = log_filepath.expanduser()
        self.timings: dict[str, list[Measurement]] = defaultdict(list)
        self.log_position: int | None = None

        on_repeat(self.step, 10)

    async def step(self) -> None:
        if not self.log_filepath.is_file():
            return

        if self.log_position is None:  # NOTE only messages generated after startup should be analyzed
            self.log_position = self.log_filepath.stat().st_size
        if self.log_filepath.stat().st_size < self.log_position:  # NOTE detect beginning of new log file
            self.log_position = 0
        ignore_warnings = False
        with self.log_filepath.open() as f:
            f.seek(self.log_position)
            for line in f:
                colored_line = color_pattern.sub('', line)
                if 'start garbage collection' in colored_line:
                    ignore_warnings = True
                if 'finished garbage collection' in colored_line:
                    ignore_warnings = False
                if not ignore_warnings:
                    message = self.parse_async_warning(colored_line)
                    if message:
                        self.timings[message.name].append(message)
            self.log_position = f.tell()

    def parse_async_warning(self, msg: str) -> Measurement | None:
        if 'rosys/actors/asyncio_monitor.py' in msg:
            return None  # NOTE we ignore our own messages
        match_warning = warning_pattern.match(msg)
        if match_warning is None:
            return None
        time = match_warning.group(1)
        duration = float(match_warning.group(3))
        description = match_warning.group(2)
        if 'TimerHandle' in description:
            name = 'TimerHandle'
            details = description
        elif '<Handle' in description:
            name = 'Handle'
            details = description
        else:
            match_task = task_pattern.match(description)
            if match_task is None:
                self.log.warning('could not parse task "%s"', description)
                return None
            name = match_task.group(1)
            details = match_task.group(2)
            if name.startswith('Task-'):
                match_coro = coro_pattern.match(details)
                if match_coro is not None:
                    name = match_coro.group(1)
                    details = match_coro.group(2)
        return Measurement(time=time, name=name, duration=duration, details=details)
