
from enum import Enum


class Actor:

    class State(str, Enum):
        startup = 'startup'
        running = 'running'
        stopping = 'stopping'
        stopped = 'stopped'

    interval: float = None
    state: State = State.running

    async def step(self):
        pass
