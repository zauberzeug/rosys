import os
import re
from collections import defaultdict
from dataclasses import dataclass

from .actor import Actor
from .garbage_collector import GarbageCollector


@dataclass
class Measurement:
    time: str
    duration: float
    name: str
    details: str


color_pattern = re.compile(r'(?:\x1B[@-_]|[\x80-\x9F])[0-?]*[ -/]*[@-~]')
warning_pattern = re.compile(r"(.*) \[WARNING\].*Executing(.*)took (.*) seconds")
task_pattern = re.compile(r".*name=['\"](.*)['\"] coro=<(.*)> .*")
coro_pattern = re.compile(r"(.*) (running at .*)")


class AsyncioMonitor(Actor):
    interval: float = 10

    def __init__(self) -> None:
        super().__init__()
        self.timings: dict[str, list[Measurement]] = defaultdict(list)
        self.log_position = None

    async def step(self):
        await super().step()
        logfile = os.path.expanduser('~/.rosys/debug.log')
        if not os.path.isfile(logfile):
            return
        self.parse_log(logfile)

    def parse_log(self, log):
        if self.log_position is None:  # NOTE only messgages generated after startup should be analyzed
            self.log_position = os.path.getsize(log)
        if os.path.getsize(log) < self.log_position:  # NOTE detect beginning of new log file
            self.log_position = 0
        ignore_warnings = False
        with open(log, 'r') as f:
            f.seek(self.log_position)
            for line in f:
                line = color_pattern.sub('', line)
                if GarbageCollector.starting_msg in line:
                    ignore_warnings = True
                if GarbageCollector.finished_msg in line:
                    ignore_warnings = False
                if not ignore_warnings:
                    message = self.parse_async_warning(line)
                    if message:
                        self.timings[message.name].append(message)
            self.log_position = f.tell()

    def parse_async_warning(self, msg: str):
        if 'rosys/actors/asyncio_monitor.py' in msg:
            return  # NOTE we ignore our own messages
        match_warning = warning_pattern.match(msg)
        if match_warning is None:
            return
        time = match_warning.group(1)
        duration = float(match_warning.group(3))
        description = match_warning.group(2)
        if 'TimerHandle' in description:
            name = 'TimerHandle'
            details = description
        else:
            match_task = task_pattern.match(description)
            if match_task is None:
                self.log.warning(f'could not parse task "{description}"')
                return
            name = match_task.group(1)
            details = match_task.group(2)
            if name.startswith('Task-'):
                match_coro = coro_pattern.match(details)
                if match_coro is not None:
                    name = match_coro.group(1)
                    details = match_coro.group(2)
        return Measurement(time=time, name=name, duration=duration, details=details)
