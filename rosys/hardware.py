from pydantic import BaseModel
from typing import Optional
import asyncio
import numpy as np
from .communication import Communication, CommunicationFactory
from .world import Mode, Velocity, World


class Simulation(BaseModel):
    linear_velocity: float = 0
    angular_velocity: float = 0


class Hardware:

    def __init__(self, world: World, communication: Optional[Communication] = ...):
        self.world = world
        self.simulation = Simulation()
        if communication is ...:
            self.communication = CommunicationFactory.create() if world.mode == Mode.REAL else None
        else:
            self.communication = communication

    async def configure(self, filepath: str):
        if self.communication:
            with open(filepath) as f:
                expander = False
                await self.communication.send_async(f'!-')
                for line in f.read().splitlines():
                    if line == '---':
                        expander = True
                        await self.communication.send_async('!>!-')
                    else:
                        if expander:
                            await self.communication.send_async(f'!>!+{line}')
                        else:
                            await self.communication.send_async(f'!+{line}')
                if expander:
                    await self.communication.send_async(f'!>!.')
                    await self.communication.send_async(f'!>core.restart()')
                await self.communication.send_async(f'!.')
                await self.restart()
        else:
            raise Exception('there is no communication for configuring hardware')

    async def restart(self):
        if self.communication:
            await self.communication.send_async(f'core.restart()')
        else:
            raise Exception('there is no communication for restarting hardware')

    async def drive(self, linear: float, angular: float):
        if self.communication:
            await self.communication.send_async(f'wheels.speed({linear}, {angular})')
        else:
            self.simulation.linear_velocity = linear
            self.simulation.angular_velocity = angular
            await asyncio.sleep(0)

    async def stop(self):
        if self.communication:
            await self.communication.send_async('wheels.off()')
        else:
            self.simulation.linear_velocity = 0
            self.simulation.angular_velocity = 0
            await asyncio.sleep(0)

    async def update(self):
        if self.communication:
            millis = None
            while True:
                line = await self.communication.read()
                if line is None:
                    break
                words = line.split()
                if not words:
                    continue
                first = words.pop(0)
                if first not in ['core', '!"core']:
                    continue
                millis = float(words.pop(0))
                if self.world.robot.clock_offset is None:
                    continue
                self.world.robot.hardware_time = millis / 1000 + self.world.robot.clock_offset
                self.parse(words)
            if millis is not None:
                self.world.robot.clock_offset = self.world.time - millis / 1000
        else:
            self.simulate()

    def parse(self, words: list[str]):
        self.world.robot.odometry.append(Velocity(
            linear=float(words.pop(0)),
            angular=float(words.pop(0)),
            time=self.world.robot.hardware_time,
        ))

    def simulate(self):
        self.world.robot.odometry.append(Velocity(
            linear=self.simulation.linear_velocity,
            angular=self.simulation.angular_velocity,
            time=self.world.time,
        ))
        self.world.robot.battery = 25.0 + np.sin(0.1 * self.world.time) + 0.02 * np.random.randn()
        self.world.robot.temperature = np.random.uniform(34, 35)
