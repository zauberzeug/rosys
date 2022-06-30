import abc
import logging
from typing import Optional

from .. import event
from ..actors.odometer import Odometer


class Wheels(abc.ABC):

    def __init__(self, odometer: Odometer) -> None:
        self.log = logging.getLogger(__name__)
        self.odometer = odometer

        event.register(event.Id.AUTOMATION_PAUSED, self.handle_stop_event)
        event.register(event.Id.AUTOMATION_STOPPED, self.handle_stop_event)
        event.register(event.Id.AUTOMATION_FAILED, self.handle_stop_event)

    async def handle_stop_event(self, _: Optional[str]) -> None:
        await self.stop()

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        pass

    @abc.abstractmethod
    async def stop(self, because: Optional[str] = None) -> None:
        pass
