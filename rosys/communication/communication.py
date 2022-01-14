from typing import Optional
import abc
import logging


class Communication(abc.ABC):

    def __init__(self) -> None:
        self.name = __name__[:-13] + self.__class__.__name__
        self.log = logging.getLogger(self.name)
        super().__init__()

    @classmethod
    @abc.abstractmethod
    def is_possible(cls) -> bool:
        return False

    @abc.abstractmethod
    async def send_async(self, msg: str):
        pass

    @abc.abstractmethod
    async def read(self) -> Optional[str]:
        pass
