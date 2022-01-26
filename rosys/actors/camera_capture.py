import cv2
import re
import shutil
from .actor import Actor
from ..world.camera import Camera, Image
from .. import event


class CameraCapture(Actor):
    interval: float = 0.30

    def __init__(self):
        super().__init__()
        self.devices = {}  # mapping camera ids to opencv devices
        self.last_scan = None

    async def step(self):
        await super().step()
        await self.update_device_list()
        for uid, camera in self.world.cameras.items():
            if not camera.capture:
                return
            bytes = await self.run_io_bound(self.capture_image, uid)
            camera.images.append(Image(camera_id=uid, data=bytes, time=self.world.time))
        self.purge_old_images()

    def capture_image(self, id) -> bytes:
        _, image = self.devices[id].read()
        return cv2.imencode('.jpg', image)[1].tobytes()

    def purge_old_images(self):
        for camera in self.world.cameras.values():
            while camera.images and camera.images[0].time < self.world.time - 5 * 60.0:
                del camera.images[0]

    async def update_device_list(self):
        if self.last_scan is not None and self.world.time < self.last_scan + 30:  # scan every 30 sec
            return
        self.last_scan = self.world.time
        output = await self.run_sh(['v4l2-ctl', '--list-devices'])
        for line in output.splitlines():
            if 'Cannot open device' in line:
                break
            if 'Camera' in line:
                uid = re.search('\((.*)\)', line).group(1)
                if uid not in self.world.cameras:
                    self.world.cameras[uid] = Camera(id=uid)
                    self.log.info(f'adding camera {uid}')
                    await event.call(event.Id.NEW_CAMERA, self.world.cameras[uid])
            if '/dev/video' in line:
                num = int(line.strip().lstrip('/dev/video'))
                if uid not in self.devices:
                    device = self.get_capture_device(num)
                    if device is None:
                        if uid in self.world.cameras:
                            del self.world.cameras[uid]
                    else:
                        self.devices[uid] = device

    def get_capture_device(self, index: int):
        try:
            capture = cv2.VideoCapture(index)
            if capture is None or not capture.isOpened():
                self.log.error(f'{index} is unavailable device')
            else:
                return capture
        except:
            self.log.exception(f'{index} device failed')

    async def tear_down(self):
        await super().tear_down()
        for capture in self.devices.values():
            capture.release()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
