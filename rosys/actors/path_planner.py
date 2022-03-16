import asyncio
import copy
from multiprocessing import Pipe
import time
from typing import Any, Optional
from .. import run
from ..world import Pose, Spline
from . import Actor
from .pathplanning.planner_process import PlannerProcess


class PathPlanner(Actor):

    def __init__(self):
        super().__init__()
        self.connection, process_connection = Pipe()
        self.process = PlannerProcess(process_connection, self.world.robot.shape.outline)
        self.last_obstacles = []
        self.last_areas = []

    async def startup(self):
        await super().startup()
        self.process.start()

    async def tear_down(self):
        await super().tear_down()
        if self.process.is_alive():
            self.process.kill()

    async def search_async(self, *, goal: Pose, start: Optional[Pose] = None, backward: bool = False, timeout: float = 3.0):
        if start is None:
            start = self.world.robot.prediction
        await self._sync_map(goal=goal, start=start)
        result = await self._call(('search', goal, start, backward, timeout))
        if result is None:
            raise TimeoutError('Could not find a path')
        else:
            return result

    async def test_spline(self, spline: Spline):
        return await self._call(('test_spline', spline))

    async def _sync_map(self, *, goal: Pose, start: Pose):
        obstacles = list(self.world.obstacles.values())
        if self.last_obstacles != obstacles:
            await self._call(('set_obstacles', obstacles))
            self.last_obstacles = copy.deepcopy(obstacles)
        areas = list(self.world.areas.values())
        if self.last_areas != areas:
            await self._call(('set_areas', areas))
            self.last_areas = copy.deepcopy(areas)
        await self._call(('update_obstacle_map', goal, start))

    async def _call(self, cmd: tuple, timeout: float = 5.0) -> Any:
        with run.cpu():
            self.connection.send(cmd)
            t = time.time()
            while not self.connection.poll():
                if time.time() - t > timeout:
                    raise TimeoutError(f'process call "{cmd[0]}" took too long')
                await asyncio.sleep(0.01)
            return self.connection.recv()
