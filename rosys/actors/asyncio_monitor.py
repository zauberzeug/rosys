from dataclasses import dataclass
import re
import shutil
import os
import rosys
from .actor import Actor


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
        self.timings: dict[str, list[Measurement]] = {}
        self.color_pattern = re.compile(r'''((\[\d+m){1,2})(?=\[[A-Z]+\])''')  # https://stackoverflow.com/a/61235364
        self.warning_pattern = re.compile(
            r"(.*) \[WARNING\].*Task pending name='(.*)' coro=<(.*)> .* took (.*) seconds")

    async def step(self):
        await super().step()
        log = os.path.expanduser('~/.rosys/debug.log')
        if not os.path.isfile(log):
            return
        with open(log, 'r') as f:
            warnings = list(filter(lambda line: '[WARNING] asyncio/base_events.py' in line, f))
            for warning in warnings[-3:]:
                warning = self.color_pattern.sub('', warning)
                result = self.warning_pattern.match(warning)
                millis = int(float(result.group(4))*1000)
                m = Measurement(time=result.group(1), name=result.group(2), millis=millis, details=result.group(3))
