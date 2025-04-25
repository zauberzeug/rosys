import asyncio
import logging
import time
from copy import copy
from multiprocessing import Pipe
from typing import Any

from .. import persistence, rosys, run
from ..driving import PathSegment
from ..event import Event
from ..geometry import Point, Pose, Prism, Spline
from .area import Area
from .obstacle import Obstacle
from .planner_process import (
    PlannerCommand,
    PlannerGrowMapCommand,
    PlannerObstacleDistanceCommand,
    PlannerProcess,
    PlannerResponse,
    PlannerSearchCommand,
    PlannerTestCommand,
)


class PathPlanner(persistence.Persistable):
    """This module runs a path planning algorithm in a separate process.

    If given, the algorithm respects the given robot shape as well as a dictionary of accessible areas and a dictionary of obstacles, both of which a backed up and restored automatically.
    The path planner can search paths, check if a spline interferes with obstacles and get the distance of a pose to any obstacle.
    """

    def __init__(self, robot_shape: Prism) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.path_planner')

        self.connection, process_connection = Pipe()
        self.process = PlannerProcess(process_connection, robot_shape.outline)
        self.responses: dict[str, Any] = {}

        self.obstacles: dict[str, Obstacle] = {}
        self.areas: dict[str, Area] = {}

        self.OBSTACLES_CHANGED = Event[dict[str, Obstacle]]()
        self.AREAS_CHANGED = Event[list[Area] | None]()

        rosys.on_startup(self.startup)
        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.step, 0.1)

    def backup_to_dict(self) -> dict:
        finished_areas = {area_id: copy(area).close() for area_id, area in self.areas.items() if len(area.outline) >= 3}
        return {
            'obstacles': persistence.to_dict(self.obstacles),
            'areas': persistence.to_dict(finished_areas),
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self.obstacles, Obstacle, data.get('obstacles', {}))
        persistence.replace_dict(self.areas, Area, data.get('areas', {}))
        self.AREAS_CHANGED.emit(None)
        self.OBSTACLES_CHANGED.emit(self.obstacles)

    def startup(self) -> None:
        self.process.start()

    async def shutdown(self) -> None:
        self.log.info('stopping planner process...')
        self.connection.close()
        self.process.connection.close()
        self.process.join(5)
        if self.process.is_alive():
            self.process.terminate()
        elif self.process.exitcode:
            self.log.info('bad exitcode for process: %s', self.process.exitcode)
        self.log.info('teardown of %s completed', self.process)

    async def step(self) -> None:
        try:
            if self.connection.poll():
                response = self.connection.recv()
                assert isinstance(response, PlannerResponse)
                if time.time() < response.deadline:
                    self.responses[response.id] = response.content
        except OSError as e:
            if 'handle is closed' in str(e):
                self.log.info('path planner process connection closed')
            else:
                raise

    async def grow_map(self, points: list[Point], timeout: float = 3.0) -> None:
        return await self._call(PlannerGrowMapCommand(
            points=points,
            deadline=time.time()+timeout,
        ))

    async def search(self, *, start: Pose, goal: Pose, timeout: float = 3.0) -> list[PathSegment]:
        return await self._call(PlannerSearchCommand(
            areas=list(self.areas.values()),
            obstacles=list(self.obstacles.values()),
            start=start,
            goal=goal,
            deadline=time.time()+timeout,
        ))

    async def test_spline(self, spline: Spline, timeout: float = 3.0) -> bool:
        return await self._call(PlannerTestCommand(
            areas=list(self.areas.values()),
            obstacles=list(self.obstacles.values()),
            spline=spline,
            deadline=time.time()+timeout,
        ))

    async def get_obstacle_distance(self, pose: Pose, timeout: float = 3.0) -> float:
        return await self._call(PlannerObstacleDistanceCommand(
            areas=list(self.areas.values()),
            obstacles=list(self.obstacles.values()),
            pose=pose,
            deadline=time.time()+timeout,
        ))

    async def _call(self, command: PlannerCommand, check_interval: float = 0.1) -> Any:
        with run.cpu():
            self.connection.send(command)
            while command.id not in self.responses:
                if time.time() > command.deadline:
                    raise TimeoutError(f'process call {command.id} did not respond in time')
                if rosys.is_test:
                    await self.step()  # NOTE: otherwise step() is not called while awaiting response
                await asyncio.sleep(check_interval)
            result = self.responses.pop(command.id)
            if isinstance(result, Exception):
                raise result
            return result
