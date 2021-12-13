from typing import Optional
import abc


class Communication(abc.ABC):

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
