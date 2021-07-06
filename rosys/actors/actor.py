
from enum import Enum
import logging


class Actor:

    class State(str, Enum):
        inactive = 'inactive'
        activating = 'activating'
        active = 'active'
        deactivating = 'deactivating'
        paused = 'paused'

    interval: float = None
    state: State = State.inactive

    async def step(self):
        pass

    async def activating(self, next_state: State = State.active):
        self.state = Actor.State.active

    async def deactivating(self):
        self.state = Actor.State.inactive

    def pause(self):
        if self.state == Actor.state.active:
            self.state = Actor.State.paused

    def resume(self):
        if self.state == Actor.state.paused:
            self.state = Actor.State.active
