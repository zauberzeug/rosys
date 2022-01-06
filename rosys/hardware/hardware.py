from ..world import Mode, Velocity, World
from ..communication import Communication, CommunicationFactory, DummyCommunication
import numpy as np
import asyncio
from pydantic import BaseModel
from typing import Optional
import logging


class Simulation(BaseModel):
    linear_velocity: float = 0
    angular_velocity: float = 0


class Hardware:

    def __init__(self, world: World, communication: Optional[Communication] = ...):
        self.world = world
        self.name = __name__[:-8] + self.__class__.__name__
        self.log = logging.getLogger(self.name)
        self.simulation = Simulation()
        if communication is ...:
            self.communication = CommunicationFactory.create() if world.mode == Mode.REAL else DummyCommunication()
        else:
            self.communication = communication

    async def configure(self):
        '''Send current configuration to the hardware.'''
        if type(self.communication) == DummyCommunication:
            raise Exception('there is no communication for configuring hardware')

    async def restart(self):
        '''Restarts hardware'''
        if type(self.communication) == DummyCommunication:
            raise Exception('there is no communication for restarting hardware')

    async def drive(self, linear: float, angular: float):
        '''Send drive command to the hardware'''
        if type(self.communication) == DummyCommunication:
            self.simulation.linear_velocity = linear
            self.simulation.angular_velocity = angular
            await asyncio.sleep(0)

    async def stop(self):
        '''Stop the driving wheels'''
        if type(self.communication) == DummyCommunication:
            self.simulation.linear_velocity = 0
            self.simulation.angular_velocity = 0
            await asyncio.sleep(0)

    async def update(self):
        if type(self.communication) == DummyCommunication:
            self.world.robot.odometry.append(Velocity(
                linear=self.simulation.linear_velocity,
                angular=self.simulation.angular_velocity,
                time=self.world.time,
            ))
            self.world.robot.battery = 25.0 + np.sin(0.1 * self.world.time) + 0.02 * np.random.randn()
            self.world.robot.temperature = np.random.uniform(34, 35)
