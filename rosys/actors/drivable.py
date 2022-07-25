from typing import Protocol


class Drivable(Protocol):

    async def drive(self, linear_speed: float, angular_speed: float) -> None:
        ...
