import abc
import logging

import rosys

from ..driving import Odometer


class Wheels(abc.ABC):

    def __init__(self, odometer: Odometer) -> None:
        self.log = logging.getLogger(__name__)

        self.odometer = odometer

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        pass

    @abc.abstractmethod
    async def stop(self) -> None:
        pass
