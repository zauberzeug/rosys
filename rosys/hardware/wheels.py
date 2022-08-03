import abc
import logging

import rosys

from ..event import Event


class Wheels(abc.ABC):
    VELOCITY_MEASURED = Event()
    '''new velocity measurements are available for processing (argument: list of velocities)'''

    def __init__(self) -> None:
        self.log = logging.getLogger(__name__)

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        pass

    @abc.abstractmethod
    async def stop(self) -> None:
        pass
