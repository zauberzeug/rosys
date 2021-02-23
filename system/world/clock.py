from pydantic import BaseModel


class Clock(BaseModel):

    time: float = 0
    interval: float

    def loop(self):

        self.time += self.interval
