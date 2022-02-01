from .actor import Actor
from ..world import Image, UsbCamera
from .. import event


class UsbCameraSimulator(Actor):
    interval: float = 1

    async def step(self):
        await super().step()
        while len(self.world.usb_cameras) < 2:
            uid = f'simulated_cam_{len(self.world.usb_cameras)}'
            self.world.usb_cameras[uid] = UsbCamera(id=uid)
            await event.call(event.Id.NEW_CAMERA, self.world.usb_cameras[uid])

        for camera in self.world.usb_cameras.values():
            camera.images.append(Image.create_placeholder(f'{camera.id}: {self.world.time:.2f}', time=self.world.time))
