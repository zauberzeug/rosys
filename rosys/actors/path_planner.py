import asyncio
import os
import signal
import time
from multiprocessing import Pipe
from typing import Any, Optional

import psutil
import rosys

from .. import run
from ..helpers import is_test
from ..world import PathSegment, Point, Pose, Spline
from . import Actor
from .pathplanning import (PlannerCommand, PlannerGrowMapCommand, PlannerObstacleDistanceCommand, PlannerProcess,
                           PlannerResponse, PlannerSearchCommand, PlannerTestCommand)


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
        self.log.info('stopping planner process...')
        pid = self.process.pid
        if self.process.is_alive():
            self.process.kill()
        # to really make sure it's gone (see https://trello.com/c/M9IvOg1c/698-reload-klappt-nicht-immer#comment-62aaeb74672e6759fba37b40)
        if not rosys.is_test:
            while pid is not None and psutil.pid_exists(pid):
                self.log.info(f'{pid} still exists; killing again')
                os.kill(pid, signal.SIGKILL)
                await rosys.sleep(1)
        self.log.info(f'teardown of {self.process} completed ({self.process.is_alive()})')

        self.connection.close()
        self.process.connection.close()

    async def step(self):
        if self.connection.poll():
            response = self.connection.recv()
            assert isinstance(response, PlannerResponse)
            if time.time() < response.deadline:
                self.responses[response.id] = response.content

    async def grow_map(self, points: list[Point], timeout: float = 3.0) -> None:
        return await self._call(PlannerGrowMapCommand(
            points=points,
            deadline=time.time()+timeout,
        ))

    async def search(self, *,
                     goal: Pose,
                     start: Optional[Pose] = None,
                     timeout: float = 3.0) -> list[PathSegment]:
        return await self._call(PlannerSearchCommand(
            areas=list(self.world.areas.values()),
            obstacles=list(self.world.obstacles.values()),
            start=start or self.world.robot.prediction,
            goal=goal,
            deadline=time.time()+timeout,
        ))

    async def test_spline(self, spline: Spline, timeout: float = 3.0) -> bool:
        return await self._call(PlannerTestCommand(
            areas=list(self.world.areas.values()),
            obstacles=list(self.world.obstacles.values()),
            spline=spline,
            deadline=time.time()+timeout,
        ))

    async def get_obstacle_distance(self, pose: Pose, timeout: float = 3.0) -> float:
        return await self._call(PlannerObstacleDistanceCommand(
            areas=list(self.world.areas.values()),
            obstacles=list(self.world.obstacles.values()),
            pose=pose,
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
