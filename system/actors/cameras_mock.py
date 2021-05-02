from world.camera import Camera
from world.world import World
from actors.actor import Actor


class CamerasMock(Actor):

    interval: float = 5.0

    async def step(self, world: World):

        mac = 'FF:FF'
        camera = world.cameras[mac] = world.cameras.get(mac) or Camera(mac=mac)
        camera.network.ip = 'mocked_cam'
