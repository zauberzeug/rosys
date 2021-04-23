import aioserial
from actors.actor import Actor


class Serial(Actor):

    aioserial_instance = aioserial.AioSerial('/dev/esp', baudrate=115200)

    async def step(self):

        try:
            line = (await self.aioserial_instance.readline_async()).decode().strip()
        except:
            raise IOError('Error reading from serial')

        if line.startswith("\x1b[0;32m"):
            return  # NOTE: ignore green log messages

        if '^' in line:
            line, check = line.split('^')
            checksum = 0
            for c in line:
                checksum ^= ord(c)
            if checksum != int(check):
                return

        try:
            words = line.split()[2:]
            # TODO: read millis and compoare with world time
            linear = float(words.pop(0))
            angular = float(words.pop(0))
        except (IndexError, ValueError):
            raise IOError(f'Error parsing serial message "{line}"')

        self.world.robot.velocity.linear = linear
        self.world.robot.velocity.angular = angular

    async def send(self, line):

        checksum = 0
        for c in line:
            checksum ^= ord(c)
        line += '^%d\n' % checksum
        await self.aioserial_instance.write_async(line.encode())
