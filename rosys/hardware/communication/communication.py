import abc
import logging
from typing import Optional


class Communication(abc.ABC):

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.communication')

    @classmethod
    @abc.abstractmethod
    def is_possible(cls) -> bool:
        return False

    @abc.abstractmethod
    async def send(self, msg: str) -> None:
        pass

    @abc.abstractmethod
    async def read(self) -> Optional[str]:
        pass
