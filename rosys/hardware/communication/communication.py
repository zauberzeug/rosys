import abc
import logging


class Communication(abc.ABC):
    """This abstract module defines an interface for communicating with a microcontroller.

    Besides sending and receiving messages a communication module provides a property whether communication is possible.
    It can also provide a piece of debug UI.
    """

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.communication')

    @classmethod
    @abc.abstractmethod
    def is_possible(cls) -> bool:
        return False

    def connect(self) -> None:  # noqa: B027
        pass

    def disconnect(self) -> None:  # noqa: B027
        pass

    @abc.abstractmethod
    async def send(self, msg: str) -> None:
        pass

    @abc.abstractmethod
    async def read(self) -> str | None:
        pass

    def debug_ui(self) -> None:  # noqa: B027
        pass
