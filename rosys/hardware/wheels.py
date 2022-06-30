import abc
import logging

from .. import event
from ..actors.odometer import Odometer


class Wheels(abc.ABC):

    def __init__(self, odometer: Odometer) -> None:
        self.log = logging.getLogger(__name__)
        self.odometer = odometer

        event.register(event.Id.AUTOMATION_PAUSED, lambda _: self.stop())
        event.register(event.Id.AUTOMATION_STOPPED, lambda _: self.stop())
        event.register(event.Id.AUTOMATION_FAILED, lambda _: self.stop())

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        pass

    @abc.abstractmethod
    async def stop(self) -> None:
        pass
