import asyncio
from multiprocessing import Pipe
import time
from typing import Any, Optional

from .. import run
from ..world import PathSegment, Point, Pose, Spline
from ..helpers import is_test
from . import Actor
from .pathplanning import PlannerCommand, PlannerProcess, PlannerResponse, PlannerState, \
    PlannerGetStateCommand, PlannerGrowMapCommand, PlannerSearchCommand, PlannerTestCommand


class PathPlanner(Actor):
    interval: float = 0.1

    def __init__(self):
        super().__init__()
        self.connection, process_connection = Pipe()
        self.process = PlannerProcess(process_connection, self.world.robot.shape.outline)
        self.responses: dict[str, Any] = {}

    async def startup(self):
        await super().startup()
        self.process.start()

    async def tear_down(self):
        await super().tear_down()
        if self.process.is_alive():
            self.process.kill()

    async def step(self):
        if self.connection.poll():
            response = self.connection.recv()
            assert isinstance(response, PlannerResponse)
            if time.time() < response.deadline:
                self.responses[response.id] = response.content

    async def get_state(self, timeout: float = 3.0) -> PlannerState:
        return await self._call(PlannerGetStateCommand(
            deadline=time.time()+timeout,
        ))

    async def grow_map(self, points: list[Point], timeout: float = 3.0) -> None:
        return await self._call(PlannerGrowMapCommand(
            points=points,
            deadline=time.time()+timeout,
        ))

    async def search(self, *,
                     goal: Pose,
                     start: Optional[Pose] = None,
                     backward: bool = False,
                     timeout: float = 3.0) -> list[PathSegment]:
        return await self._call(PlannerSearchCommand(
            areas=list(self.world.areas.values()),
            obstacles=list(self.world.obstacles.values()),
            start=start or self.world.robot.prediction,
            goal=goal,
            backward=backward,
            deadline=time.time()+timeout,
        ))

    async def test_spline(self, spline: Spline, timeout: float = 3.0):
        return await self._call(PlannerTestCommand(
            areas=list(self.world.areas.values()),
            obstacles=list(self.world.obstacles.values()),
            spline=spline,
            deadline=time.time()+timeout,
        ))

    async def _call(self, command: PlannerCommand) -> Any:
        with run.cpu():
            self.connection.send(command)
            while command.id not in self.responses:
                if time.time() > command.deadline:
                    raise TimeoutError(f'process call {command.id} did not respond in time')
                if is_test:
                    await self.step()  # NOTE: otherwise step() is not called while awaiting response
                await asyncio.sleep(self.interval)
            result = self.responses.pop(command.id)
            if isinstance(result, Exception):
                raise result
            return result
