import asyncio
import subprocess
from actors.actor import Actor
from world.camera import Camera
from world.world import World


class CommandNotFound(Exception):
    pass


class CameraScanner(Actor):

    interval: float = 5.0

    def __init__(self):
        super().__init__()
        self.device = subprocess.run(
            'iw dev | awk \'$1=="Interface"{print $2}\' | sort | head -n 1',
            shell=True, stdout=subprocess.PIPE,
        ).stdout.decode('utf-8').rstrip()

    async def step(self, world: World):

        output = await self.subprocess(f'iw dev {self.device} station dump')
        for line in output.splitlines():
            if "iw: not found" in line:
                raise CommandNotFound('iw')
            if line.startswith("Station"):
                mac = line.split()[1]
                world.cameras[mac] = world.cameras.get(mac) or Camera(mac=mac)
                camera = world.cameras[mac]
            if line.strip().startswith("inactive time:"):
                camera.network.inactive_time = float(line.split()[-2]) / 1000
            if line.strip().startswith("expected throughput:") and mac is not None:
                camera.network.expected_throughput = float(line.split()[-1].replace('Mbps', ''))
            if line.strip().startswith("signal:") and mac is not None:
                camera.network.signal = float(line.split()[-2])
            if line.strip().startswith("signal avg:") and mac is not None:
                camera.network.signal_average = float(line.split()[-2])

        for mac, camera in world.cameras.items():
            output = await self.subprocess(f'ip neigh | grep {mac}')
            if "ip: not found" in line:
                raise CommandNotFound('ip')
            try:
                camera.network.ip = output.split()[0]
            except IndexError as e:
                print(e)

    async def subprocess(self, command: str) -> str:

        proc = await asyncio.create_subprocess_shell(
            command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT)
        stdout, *_ = await proc.communicate()
        return stdout.decode()
