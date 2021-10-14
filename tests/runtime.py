import asyncio
from rosys.runtime import Runtime
from rosys.world.marker import Marker
from rosys.world.mode import Mode
from rosys.world.robot import Robot
from rosys.world.world import World, WorldState


class TestRuntime(Runtime):

    def __init__(self):
        robot = Robot(marker=Marker(points={'front': (0.12, 0), 'back': (-0.12, 0)}, height=0.58))
        world = World(mode=Mode.TEST, state=WorldState.RUNNING, robot=robot)
        super().__init__(world)
        self.with_cameras()
        self.world.set_time(0)  # NOTE in tests we start at zero for better reading

        from tests.helper import set_global_runtime  # NOTE import here to avoid PytestAssertRewriteWarning
        set_global_runtime(self)

        for actor in self.actors:
            actor.sleep = self.sleep

    async def forward(self, seconds, dt=0.01):
        # NOTE we start runtime here because this makes it easy in the tests to prepare it beforehand
        if not self.tasks:
            await self.run()

        end_time = self.world.time + seconds
        while self.world.time <= end_time:
            self.world.set_time(self.world.time + dt)
            await asyncio.sleep(0)

    async def sleep(self, seconds: float):
        sleep_end_time = self.world.time + seconds
        while self.world.time <= sleep_end_time:
            await asyncio.sleep(0)
