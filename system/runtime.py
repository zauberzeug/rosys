from actors.actor import Actor
from actors.detector import Detector
import icecream
import sys
import time
import asyncio
import task_logger
import logging
from typing import get_type_hints
from actors.esp import SerialEsp, MockedEsp
from actors.automator import Automator
from actors.camera_scanner import CameraScanner, NotFound
from actors.camera_downloader import CameraDownloader
from actors.cameras_mock import CamerasMock
from world.world import World
from world.robot import Robot
from world.state import State
from world.mode import Mode


class Runtime:

    def __init__(self, mode: Mode):

        self.world = World(
            mode=mode,
            state=State.RUNNING,
            time=time.time(),
            robot=Robot(),
            cameras=[],
        )
        self.esp = SerialEsp() if mode == Mode.REAL else MockedEsp()
        self.automator = Automator()

        self.actors = [
            self.esp,
            self.automator,
        ]

        if mode == Mode.REAL:
            self.camera_scanner = CameraScanner()
            self.camera_downloader = CameraDownloader()
            self.detector = Detector()

            self.actors.extend([self.camera_scanner, self.camera_downloader, self.detector])
        else:
            self.cameras_mock = CamerasMock()
            self.actors.append(self.cameras_mock)

    async def pause(self):
        self.world.state = State.PAUSED
        await self.esp.drive(0, 0)

    def resume(self):
        self.world.state = State.RUNNING

    async def run(self, seconds: float = sys.maxsize):

        end_time = self.world.time + seconds
        tasks = [task_logger.create_task(self.advance_time(end_time))]

        for actor in self.actors:
            tasks.append(task_logger.create_task(self.repeat(actor, end_time)))

        await asyncio.gather(*tasks)

    async def repeat(self, actor: Actor, run_end_time: float):

        params = self.get_params(actor.step)

        while self.world.time < run_end_time:

            start = time.time()
            await actor.step(*params)
            dt = time.time() - start

            interval = actor.interval

            if actor.interval == 0 and dt < 0.1:
                logging.warning(
                    f'{type(actor).__name__} would be called to frequently {dt} s; delaying this step for 100 ms')
                interval = 0.1
            elif dt > actor.interval > 0:
                logging.warning(f'{type(actor).__name__} took {dt} s')

            if self.world.mode == Mode.TEST:
                sleep_end_time = self.world.time + interval
                while self.world.time < min(run_end_time, sleep_end_time):
                    await asyncio.sleep(0)
            else:
                await asyncio.sleep(interval - dt)

    async def advance_time(self, end_time):

        while True:
            self.world.time = self.world.time + 0.01 if self.world.mode == Mode.TEST else time.time()
            if self.world.time > end_time:
                break
            await asyncio.sleep(0 if self.world.mode == Mode.TEST else 0.01)

    def get_params(self, func):

        params = []
        for name, type_ in get_type_hints(func).items():
            for obj in [self.world] + self.actors:
                if isinstance(obj, type_):
                    params.append(obj)
                    break
            else:
                raise Exception(f'parameter "{name}" of type {type_} is unknown')

        return params
