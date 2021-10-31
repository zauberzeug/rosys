from __future__ import annotations
from asyncio.exceptions import CancelledError
from rosys import factory
import sys
import asyncio
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


class Runtime:

    def __init__(self, world: World = None, persistence: Optional[Persistence] = None):
        self.world = world or World()
        self.tasks = []
        self.log = logging.getLogger(__name__)

        if self.world.mode != Mode.TEST:
            self.persistence = persistence or Persistence(self.world)
            self.persistence.restore()

        self.esp = factory.create_esp(self.world)
        self.log.info(f'selected {type(self.esp).__name__}')
        self.odometer = Odometer()
        self.steerer = Steerer(self.world)
        self.automator = Automator()

        self.actors = [
            self.esp,
            self.odometer,
            self.steerer,
            self.automator,
        ]

        self.follow_ups = {
            self.esp.step: [
                self.odometer.handle_velocity,
            ],
        }

    def with_actors(self, *actors: list[Actor]):
        self.actors += actors
        return self

    async def pause(self, because: str = None):
        if because:
            self.notify(f'pausing automations because {because}')
        self.world.state = WorldState.PAUSED
        await self.esp.drive(0, 0)

    def resume(self):
        self.world.state = WorldState.RUNNING

    async def start(self):
        if self.tasks:
            raise Exception('run should be only executed once')

        for actor in self.actors:
            if actor.interval is not None:
                self.tasks.append(task_logger.create_task(self.repeat(actor)))

        await asyncio.sleep(1)  # NOTE we wait for RoSys to start up before analyizing async debugging
        self.activate_async_debugging()

    async def stop(self):
        await self.esp.drive(0, 0)
        if self.world.mode != Mode.TEST:
            self.persistence.backup()
        [t.cancel() for t in self.tasks]
        await asyncio.gather(*[task_logger.create_task(a.tear_down()) for a in self.actors])

    async def call_follow_ups(self, trigger: Union[Callable, Awaitable]):
        for follow_up in self.follow_ups.get(trigger, []):
            params = self.get_params(follow_up)
            await follow_up(*params) if asyncio.iscoroutine(follow_up) else follow_up(*params)
            await self.call_follow_ups(follow_up)

    async def repeat(self, actor: Actor):
        params = self.get_params(actor.step)
        actor.pause_automations = self.pause  # NOTE injecting the correct implementation

        while True:
            start = self.world.time
            try:
                await actor.step(*params)
                await self.call_follow_ups(actor.step)
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

    def notify(self, message: str):
        '''Notify the user.

        Should be replaced and implemented by the user interface (as it's done in rosys.ui.configure).
        '''
        self.log.info(message)

    def activate_async_debugging(self):
        '''Produce warnings for coroutines which take to long on the main loop and hence clog the event loop'''
        try:
            loop = asyncio.get_running_loop()
            loop.set_debug(True)
            loop.slow_callback_duration = 0.05
        except:
            self.log.exception('could not activete async debugging')
