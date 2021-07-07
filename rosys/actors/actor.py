class Actor:

    interval: float = None

    async def step(self):
        pass

    async def tear_down(self):
        pass

    def __str__(self) -> str:
        return type(self).__name__
