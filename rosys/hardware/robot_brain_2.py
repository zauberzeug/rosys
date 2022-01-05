from ..world import Mode, Velocity, World
from .hardware import Hardware


class RobotBrain2(Hardware):

    async def configure(self):
        await super().restart()
        filepath = 'lizard.txt'
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

    async def restart(self):
        await super().restart()
        await self.communication.send_async(f'core.restart()')

    async def drive(self, linear: float, angular: float):
        super().drive(linear, angular)
        await self.communication.send_async(f'wheels.speed({linear}, {angular})')

    async def stop(self):
        await super().stop()
        await self.communication.send_async('wheels.off()')

    async def update(self):
        await super().update()
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

    def parse(self, words: list[str]):
        self.world.robot.odometry.append(Velocity(
            linear=float(words.pop(0)),
            angular=float(words.pop(0)),
            time=self.world.robot.hardware_time,
        ))
