from uuid import uuid4

from .. import event
from ..world import Image, UsbCamera
from .actor import Actor


class UsbCameraSimulator(Actor):
    interval: float = 1

    async def step(self):
        await super().step()

        for camera in self.world.usb_cameras.values():
            camera.images.append(Image.create_placeholder(f'{camera.id}: {self.world.time:.2f}', time=self.world.time))

    async def create(
        self, uid: str = '',
        x: float = 0, y: float = 0, z: float = 1,
        yaw: float = 0, tilt_x: float = 0, tilt_y: float = 0,
    ):
        if uid == '':
            uid = f'simulated_cam_{len(self.world.usb_cameras)}'
        camera = UsbCamera(id=uid)
        camera.set_perfect_calibration(x, y, z, yaw, tilt_x, tilt_y)
        self.world.usb_cameras[uid] = camera
        await event.call(event.Id.NEW_CAMERA, self.world.usb_cameras[uid])
