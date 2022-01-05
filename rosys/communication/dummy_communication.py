
from typing import Optional
from .communication import Communication


class DummyCommunication(Communication):

    def is_possible(cls) -> bool:
        return False

    async def send_async(self, msg: str):
        pass

    async def read(self) -> Optional[str]:
        pass
