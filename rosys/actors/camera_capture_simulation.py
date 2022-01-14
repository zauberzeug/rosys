from .actor import Actor
from ..world.camera import Camera, Frame
from .. import event


class CameraCaptureSimulation(Actor):
    interval: float = 1

    def __init__(self) -> None:
        super().__init__()

    async def step(self):
        await super().step()
        while len(self.world.cameras) < 2:
            uid = f'simulated_cam_{len(self.world.cameras)}'
            self.world.cameras[uid] = Camera(id=uid)
            await event.call(event.Id.NEW_CAMERA, self.world.cameras[uid])

        for camera in self.world.cameras.values():
            camera.frames.append(Frame.create_placeholder(f'{camera.id}: {self.world.time:.2f}', time=self.world.time))
