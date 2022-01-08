from .actor import Actor
from ..world.camera import Camera, Frame


class ImageCaptureSimulation(Actor):
    interval: float = 1

    def __init__(self) -> None:
        super().__init__()
        for i in range(2):
            uid = f'simulated_cam_{i}'
            self.world.cameras[uid] = Camera(id=uid)

    async def step(self):
        await super().step()
        for camera in self.world.cameras.values():
            camera.frames.append(Frame.create_placeholder(f'{camera.id}: {self.world.time:.2f}', time=self.world.time))
