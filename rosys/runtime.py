import sys
import time
import asyncio
import logging
from typing import Awaitable, Callable, Union, get_type_hints
from . import task_logger
from .actors.actor import Actor
from .actors.detector import Detector
from .actors.esp import SerialEsp, MockedEsp
from .actors.odometer import Odometer
from .actors.steerer import Steerer
from .actors.automator import Automator
from .actors.camera_scanner import CameraScanner
from .actors.camera_downloader import CameraDownloader
from .actors.cameras_mock import CamerasMock
from .world.world import World
from .world.robot import Robot
from .world.state import State
from .world.mode import Mode
from .helpers import print_stacktrace


class Runtime:

    def __init__(self, mode: Mode):

        self.world = World(
            mode=mode,
            state=State.RUNNING,
            time=time.time(),
            robot=Robot(),
            download_queue=[],
            cameras=[],
            images=[],
            image_data={},
        )
        self.esp = SerialEsp() if mode == Mode.REAL else MockedEsp()
        self.odometer = Odometer()
        self.steerer = Steerer()
        self.automator = Automator()

        self.actors = [
            self.esp,
            self.odometer,
            self.steerer,
            self.automator,
        ]

        if mode == Mode.REAL:
            self.actors.extend([
                CameraScanner(),
                CameraDownloader(),
                Detector(),
            ])
        else:
            self.actors.extend([
                CamerasMock(),
            ])

        self.follow_ups = {
            self.esp.step: [self.odometer.update_pose],
        }

    async def pause(self):
        self.world.state = State.PAUSED
        await self.esp.drive(0, 0)

    def resume(self):
        self.world.state = State.RUNNING

    async def run(self, seconds: float = sys.maxsize):

        end_time = self.world.time + seconds
        self.tasks = [task_logger.create_task(self.advance_time(end_time))]

        for actor in self.actors:
            if actor.interval is not None:
                self.tasks.append(task_logger.create_task(self.repeat(actor, end_time)))

        await asyncio.gather(*self.tasks)

    async def stop(self):

        [t.cancel() for t in self.tasks]

    async def call_targets(self, trigger: Union[Callable, Awaitable]):

        for target in self.follow_ups.get(trigger, []):
            params = self.get_params(target)
            await target(*params) if asyncio.iscoroutine(target) else target(*params)
            self.call_targets(target)

    async def repeat(self, actor: Actor, run_end_time: float):

        params = self.get_params(actor.step)

        while self.world.time < run_end_time:

            start = self.world.time
            try:
                await actor.step(*params)
                await self.call_targets(actor.step)
            except Exception as e:
                print_stacktrace(e)
            dt = self.world.time - start

            interval = actor.interval

            if actor.interval == 0 and dt < 0.1:
                interval = 0.1
                logging.info(
                    f'{type(actor).__name__} would be called to frequently ' +
                    f'because it only took {dt*1000:.0f} ms; ' +
                    f'delaying this step for {(interval - dt)*1000:.0f} ms')
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

    def get_params(self, func: Union[Callable, Awaitable]):

        params = []
        for name, type_ in get_type_hints(func).items():
            for obj in [self.world] + self.actors:
                if isinstance(obj, type_):
                    params.append(obj)
                    break
            else:
                raise Exception(f'parameter "{name}" of type {type_} is unknown')

        return params
