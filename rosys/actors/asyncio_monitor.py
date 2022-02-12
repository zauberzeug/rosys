from dataclasses import dataclass
import re
import shutil
import os
import html
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
        self.color_pattern = re.compile(r'''((\[\d+m){1,2})(?=\[[A-Z]+\])''')  # https://stackoverflow.com/a/61235364
        self.warning_pattern = re.compile(r"(.*) \[WARNING\].*Executing(.*)took (.*) seconds")
        self.task_pattern = re.compile(r".*name=['\"](.*)['\"] coro=<(.*)> .*")
        self.log_position = 0
        self.log_size = 0
        # # NOTE also parse archived logfile once
        # log1 = os.path.expanduser('~/.rosys/debug.log.1')
        # if os.path.isfile(log1):
        #     self.parse(log1)
        # # NOTE reset log file infos for latest logfile
        # self.log_position = 0
        # self.log_size = 0

    async def step(self):
        await super().step()
        ic(len(self.timings))
        log = os.path.expanduser('~/.rosys/debug.log')
        if not os.path.isfile(log):
            self.log.warning('cound not find debug.log')
            return
        self.parse(log)

    def parse(self, log):
        if os.path.getsize(log) < self.log_size:  # NOTE detect beginning of new log file
            self.log_position = 0
        self.log_size = os.path.getsize(log)
        with open(log, 'r') as f:
            self.log.info(f'parsing from pos {self.log_position}; size is {self.log_size}')
            f.seek(self.log_position)
            warnings = list(filter(lambda line: 'asyncio/base_events.py' in line, f))
            ic(len(warnings))
            for warning in warnings:
                if 'could not parse' in warning:
                    continue
                warning = self.color_pattern.sub('', warning)
                self.log.info(f'trying "{warning}"')
                match_warning = self.warning_pattern.match(warning)
                if match_warning is None:
                    self.log.warning(f'could not parse: {warning}')
                    continue
                time = match_warning.group(1)
                ic(time)
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
                ic(m)
                m = Measurement(time=time, name=self.clean(name), millis=millis, details=self.clean(details))
                self.timings[m.name].append(m)
            self.log_position = f.tell()

    def clean(self, text):
        return html.escape(text)
