from __future__ import annotations
from asyncio.exceptions import CancelledError
import asyncio
import logging
from typing import Optional, Type
from . import event, task_logger
from .actors import Actor, Automator, Lizard, Odometer, Steerer
from .hardware import Hardware
from .persistence import Persistence
from .world import AutomationState, Mode, World


class Runtime:

    def __init__(self,
                 world: Optional[World] = None,
                 persistence: Optional[Persistence] = None,
                 hardware: Optional[Hardware] = None):
        self.world = world or World()
        Actor.world = self.world
        self.tasks = []
        self.log = logging.getLogger(__name__)

        if self.world.mode != Mode.TEST:
            self.persistence = persistence or Persistence(self.world)
            self.persistence.restore()

        self.hardware = hardware or Hardware(self.world)
        self.lizard = Lizard(self.hardware)
        self.odometer = Odometer()
        self.steerer = Steerer(self.hardware)
        self.automator = Automator()

        self.actors = [
            self.lizard,
            self.odometer,
            self.steerer,
            self.automator,
        ]

    def with_actors(self, *actors: list[Actor]):
        self.actors += actors
        return self

    async def pause(self, because: Optional[str] = None):
        await event.call(event.Id.PAUSE_AUTOMATIONS, because)

    async def stop(self, because: Optional[str] = None):
        await event.call(event.Id.PAUSE_AUTOMATIONS, because)
        self.automator.routines.clear()
        self.world.automation_state = AutomationState.STOPPED

    async def resume(self):
        previous = self.world.automation_state
        if previous != AutomationState.DISABLED:
            self.world.automation_state = AutomationState.RUNNING
            if previous == AutomationState.STOPPED:
                event.emit(event.Id.AUTOMATIONS_STARTED)

    async def startup(self):
        if self.tasks:
            raise Exception('should be only executed once')

        event.register(event.Id.NEW_NOTIFICATION, self.store_notification)
        for actor in self.actors:
            if actor.interval is not None:
                self.tasks.append(task_logger.create_task(self.repeat(actor), name=actor.name))
        self.tasks.append(asyncio.create_task(self.watch_emitted_events()))

        await asyncio.sleep(1)  # NOTE we wait for RoSys to start up before analyzing async debugging
        self.activate_async_debugging()

    async def shutdown(self):
        await self.hardware.drive(0, 0)
        if self.world.mode != Mode.TEST:
            self.persistence.backup()
        [t.cancel() for t in self.tasks]
        Actor.process_pool.shutdown()
        await asyncio.gather(*[task_logger.create_task(a.tear_down(), name=f'{a.name}.tear_down()') for a in self.actors])

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
        self.log.exception('task failed to execute', exc_info=ex)
