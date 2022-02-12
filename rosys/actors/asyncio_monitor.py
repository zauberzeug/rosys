from dataclasses import dataclass
import re
import shutil
import os
import rosys
from .actor import Actor
from collections import defaultdict


@dataclass
class Measurement:
    time: str
    millis: int
    name: str
    details: str


class AsyncioMonitor(Actor):
    interval: float = 10

    def __init__(self) -> None:
        super().__init__()
        self.timings: dict[str, list[Measurement]] = defaultdict(list)
        self.log_position = 0
        self.color_pattern = re.compile(r'''((\[\d+m){1,2})(?=\[[A-Z]+\])''')  # https://stackoverflow.com/a/61235364
        self.warning_pattern = re.compile(r"(.*) \[WARNING\].*Executing(.*)took (.*) seconds")
        self.task_pattern = re.compile(r".*name=['\"](.*)['\"] coro=<(.*)> .*")

    async def step(self):
        await super().step()
        log = os.path.expanduser('~/.rosys/debug.log.1')
        if not os.path.isfile(log):
            return
        with open(log, 'r') as f:
            f.seek(self.log_position)
            warnings = list(filter(lambda line: '[WARNING] asyncio/base_events.py' in line, f))
            for warning in warnings:
                if 'could not parse' in warning:
                    continue
                warning = self.color_pattern.sub('', warning)
                match_warning = self.warning_pattern.match(warning)
                if match_warning is None:
                    self.log.warning(f'could not parse: {warning}')
                    continue
                time = match_warning.group(1)
                millis = int(float(match_warning.group(3))*1000)
                description = match_warning.group(2)
                if 'TimerHandle' in description:
                    name = 'TimerHandle'
                    details = description
                else:
                    match_task = self.task_pattern.match(description)
                    if match_task is None:
                        self.log.warning(f'could not parse task "{warning}"')
                        continue
                    name = match_task.group(1)
                    details = match_task.group(2)
                m = Measurement(time=time, name=name, millis=millis, details=match_warning.group(3))
                self.timings[m.name].append(m)
            self.log_position = f.tell()
