import abc
import logging
from typing import Optional

from ..actors.automator import Automator
from ..actors.odometer import Odometer
from ..runtime import runtime


class Wheels(abc.ABC):

    def __init__(self, odometer: Odometer) -> None:
        self.log = logging.getLogger(__name__)
        self.odometer = odometer

        Automator.AUTOMATION_PAUSED.register(self.handle_stop_event)
        Automator.AUTOMATION_STOPPED.register(self.handle_stop_event)
        Automator.AUTOMATION_FAILED.register(self.handle_stop_event)

        runtime.on_shutdown(self.stop)

    async def handle_stop_event(self, _: Optional[str]) -> None:
        await self.stop()

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        pass

    @abc.abstractmethod
    async def stop(self, because: Optional[str] = None) -> None:
        pass
