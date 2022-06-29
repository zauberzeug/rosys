import asyncio
import sys
import time as system_time

import numpy as np
from rosys.actors import Actor
from rosys.core.notification import Notification


class Core(Actor):

    def __init__(self):
        self._time = system_time.time()
        self.notifications: list[Notification] = []
        self.is_test = 'pytest' in sys.modules

    def notify(self, message: str) -> None:
        self.log.info(message)
        self.notifications.append(Notification(self.time, message))

    @property
    def time(self) -> float:
        return self._time if self.is_test else system_time.time()

    def set_time(self, value: float) -> None:
        assert self.is_test, 'only tests can change the time'
        self._time = value

    async def sleep(self, seconds: float) -> None:
        if self.is_test:
            sleep_end_time = self.time + seconds
            while self.time <= sleep_end_time:
                await asyncio.sleep(0)
        else:
            count = int(np.ceil(seconds))
            if count > 0:
                for _ in range(count):
                    await asyncio.sleep(seconds / count)
            else:
                await asyncio.sleep(0)


core = Core()
