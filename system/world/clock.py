from pydantic import BaseModel


class Clock(BaseModel):

    time: float = 0
    interval: float

    def step(self):

        self.time += self.interval
