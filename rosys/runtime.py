from __future__ import annotations

import asyncio
import logging
from asyncio.exceptions import CancelledError
from typing import Optional, Type

from . import Persistence, event, is_test, run, sleep, task_logger
from .actors import (Actor, AsyncioMonitor, Automator, Backup, CameraProjector,
                     Detector, DetectorSimulator, GarbageCollector, Lizard,
                     NetworkMonitor, Odometer, PathPlanner, Steerer,
                     UsbCameraCapture, UsbCameraSimulator)
from .communication import CommunicationFactory
from .hardware import Hardware, RobotBrain, SimulatedHardware
from .world import World


class Runtime:
    log = logging.getLogger(__name__)

    def __init__(self,
                 world: Optional[World] = None,
                 persistence: Optional[Persistence] = None,
                 hardware: Optional[Hardware] = None):
        self.world = world or World()
        Actor.world = self.world
        self.tasks = []
        if not is_test:
            self.persistence = persistence or Persistence(self.world)
            self.persistence.restore()

        self.hardware = hardware or self.create_hardware()
        if is_test:
            assert self.hardware.is_simulation, 'real hardware must not be used in tests'
        self.lizard = Lizard(self.hardware)
        self.odometer = Odometer()
        self.steerer = Steerer(self.hardware)
        self.automator = Automator()
        self.camera_projector = CameraProjector()
        self.asyncio_monitor = AsyncioMonitor()
        self.detector: Optional[Detector] = None  # NOTE can be set by runtime.with_detector()
        self.usb_camera_simulator: Optional[UsbCameraSimulator] = None  # NOTE can be set by runtime.with_usb_cameras()
        self.actors = [
            self.camera_projector,
            self.lizard,
            self.odometer,
            self.steerer,
            self.automator,
            self.asyncio_monitor,
            GarbageCollector(),
        ]
        if NetworkMonitor.is_operable():
            self.with_actors(NetworkMonitor())
        if not is_test:
            self.with_actors(Backup(self.persistence))
        self.path_planner: Optional[PathPlanner] = None

    def create_hardware(self):
        communication = CommunicationFactory.create()
        if communication is not None:
            return RobotBrain(self.world, communication)
        else:
            return SimulatedHardware(self.world)

    def with_actors(self, *actors: list[Actor]) -> Runtime:
        '''Adds list of additional actors to runtime.'''
        self.actors += actors
        return self

    def with_usb_cameras(self) -> Runtime:
        '''Adds usb camera capture actor to runtime.'''
        if UsbCameraCapture.is_operable() and not is_test:
            self.with_actors(UsbCameraCapture())
        else:
            self.usb_camera_simulator = UsbCameraSimulator()
            self.with_actors(self.usb_camera_simulator)
        return self

    def with_detector(self, real: Optional[Detector] = None, simulation: Optional[DetectorSimulator] = None) -> Runtime:
        '''Adds detector to runtime.'''
        if self.hardware.is_real and not is_test:
            self.detector = real or Detector()
        else:
            self.detector = simulation or DetectorSimulator()
        self.with_actors(self.detector)
        return self

    def with_path_planner(self) -> Runtime:
        '''Adds path planning actor to runtime.'''
        self.path_planner = PathPlanner()
        self.with_actors(self.path_planner)
        return self

    async def startup(self):
        if self.tasks:
            raise Exception('should be only executed once')

        event.register(event.Id.NEW_NOTIFICATION, self.store_notification)
        for actor in self.actors:
            try:
                await actor.startup()
            except:
                self.log.exception(f'error while starting {actor}')
                continue
            if actor.interval is not None:
                self.log.debug(f'starting actor {actor.name} with interval {actor.interval}s')
                self.tasks.append(task_logger.create_task(self.repeat(actor), name=actor.name))
        self.tasks.append(asyncio.create_task(self.watch_emitted_events()))

        if not is_test:
            await asyncio.sleep(1)  # NOTE we wait for RoSys to start up before analyzing async debugging
        self.activate_async_debugging()
        self.log.debug('startup completed')
        self.world.start_time = self.world.time

    async def shutdown(self):
        try:
            await self.hardware.drive(0, 0)
        except:
            pass
        if not is_test:
            self.persistence.backup()
        [t.cancel() for t in self.tasks]
        if not is_test:
            run.process_pool.shutdown()
        for a in self.actors:
            await a.tear_down()
        # await asyncio.gather(*[task_logger.create_task(a.tear_down(), name=f'{a.name}.tear_down()') for a in self.actors])

    async def repeat(self, actor: Actor):
        while True:
            start = self.world.time
            try:
                await actor.step()
                dt = self.world.time - start
            except (CancelledError, GeneratorExit):
                return
            except:
                dt = self.world.time - start
                self.log.exception(f'error in {actor}')
                if actor.interval == 0 and dt < 0.1:
                    delay = 0.1 - dt
                    self.log.warning(
                        f'{actor} would be called to frequently ' +
                        f'because it only took {dt*1000:.0f} ms; ' +
                        f'delaying this step for {delay*1000:.0f} ms')
                    await sleep(delay)
            try:
                await sleep(actor.interval - dt)
            except (CancelledError, GeneratorExit):
                return

    def get_actor(self, _type: Type[Actor]) -> Actor:
        return next((a for a in self.actors if type(a) is _type), None)

    def store_notification(self, message: str):
        self.log.info(message)
        self.world.notifications.append((self.world.time, message))

    def activate_async_debugging(self):
        '''Produce warnings for coroutines which take too long on the main loop and hence clog the event loop'''
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(True)
            loop.slow_callback_duration = 0.05
        except:
            self.log.exception('could not activate async debugging')

    async def watch_emitted_events(self):
        while True:
            try:
                for task in event.tasks:
                    if task.done() and task.exception():
                        self.handle_exception(task.exception())
                # cleanup finished tasks
                event.tasks = [t for t in event.tasks if not t.done()]
            except:
                self.log.exception('failing to watch emitted events')
            await sleep(0 if is_test else 0.1)

    def handle_exception(self, ex: Exception):
        self.log.exception('task failed to execute', exc_info=ex)
