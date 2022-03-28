import io
import random
import time

import PIL as pil
import rosys

from .. import event
from ..world import Image, UsbCamera
from .actor import Actor


class UsbCameraSimulator(Actor):
    interval: float = 1

    async def step(self):
        await super().step()

        for camera in self.world.usb_cameras.values():
            size = camera.calibration.intrinsics.size
            image = Image(time=self.world.time, camera_id=camera.id, size=size)
            if rosys.is_test:
                image.data = b'test data'
            else:
                image.data = await rosys.run.cpu_bound(self.create_image_data, camera)
            camera.images.append(image)

    async def create(
        self, uid: str = '',
        x: float = 0, y: float = 0, z: float = 1,
        yaw: float = 0, tilt_x: float = 0, tilt_y: float = 0,
        color=None,
    ):
        if uid == '':
            uid = f'simulated_cam_{len(self.world.usb_cameras)}'
        camera = UsbCamera(id=uid)
        camera.color = color or '#' + ('%06x' % random.randint(0, 0xFFFFFF))
        camera.set_perfect_calibration(x, y, z, yaw, tilt_x, tilt_y)
        self.world.usb_cameras[uid] = camera
        await event.call(event.Id.NEW_CAMERA, self.world.usb_cameras[uid])

    @staticmethod
    def create_image_data(camera: UsbCamera):
        size = camera.calibration.intrinsics.size
        img = pil.Image.new('RGB', size=(size.width, size.height), color=camera.color)
        d = pil.ImageDraw.Draw(img)
        text = f'{camera.id}: {time.time()}'
        d.text((img.width/2-len(text)*3, img.height/2-5), text, fill=(255, 255, 255))
        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        return img_byte_arr.getvalue()
