from __future__ import annotations
from asyncio.exceptions import CancelledError
from rosys import factory
import asyncio
import inspect
import logging
from typing import Awaitable, Callable, Optional, Type, Union, get_type_hints
from . import task_logger
from .persistence import Persistence
from .actors.actor import Actor
from .actors.odometer import Odometer
from .actors.steerer import Steerer
from .actors.automator import Automator
from .world.world import World, WorldState
from .world.mode import Mode
from . import event


class Runtime:

    def __init__(self, world: Optional[World] = None, persistence: Optional[Persistence] = None):
        self.world = world or World()
        Actor.world = self.world
        self.tasks = []
        self.log = logging.getLogger(__name__)

        if self.world.mode != Mode.TEST:
            self.persistence = persistence or Persistence(self.world)
            self.persistence.restore()

        self.esp = factory.create_esp()
        self.log.info(f'selected {type(self.esp).__name__}')
        self.odometer = Odometer()
        self.steerer = Steerer(self.esp)
        self.automator = Automator()

        self.actors = [
            self.esp,
            self.odometer,
            self.steerer,
            self.automator,
        ]

    def with_actors(self, *actors: list[Actor]):
        self.actors += actors
        return self

    async def pause(self, because: Optional[str] = None):
        if self.world.state == WorldState.PAUSED:
            return
        if because:
            await event.call(event.Id.NEW_NOTIFICATION, f'pausing automations because {because}')
        self.world.state = WorldState.PAUSED
        await self.esp.drive(0, 0)

    def resume(self):
        self.world.state = WorldState.RUNNING

    async def start(self):
        if self.tasks:
            raise Exception('should be only executed once')

        event.register(event.Id.NEW_NOTIFICATION, self.store_notification)
        event.register(event.Id.PAUSE_AUTOMATIONS, self.pause)
        for actor in self.actors:
            if actor.interval is not None:
                self.tasks.append(task_logger.create_task(self.repeat(actor)))
        self.tasks.append(asyncio.create_task(self.watch_emitted_events()))

        await asyncio.sleep(1)  # NOTE we wait for RoSys to start up before analyzing async debugging
        self.activate_async_debugging()

    async def stop(self):
        await self.esp.drive(0, 0)
        if self.world.mode != Mode.TEST:
            self.persistence.backup()
        [t.cancel() for t in self.tasks]
        Actor.process_pool.shutdown()
        await asyncio.gather(*[task_logger.create_task(a.tear_down()) for a in self.actors])

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
                    await actor.sleep(delay)

            try:
                await actor.sleep(actor.interval - dt)
            except (CancelledError, GeneratorExit):
                return

    def get_actor(self, _type: Type):
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
            await asyncio.sleep(0 if self.world.mode == Mode.TEST else 0.1)

    def handle_exception(self, ex: Exception):
        self.log.exception(ex)
#        self.log.exception('task failed to execute', exec_info=ex)
