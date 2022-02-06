import cv2
import re
import shutil
import rosys
from ..world import UsbCamera, Image, ImageSize
from .. import event
from .actor import Actor


class UsbCameraCapture(Actor):
    interval: float = 0.30

    def __init__(self):
        super().__init__()
        self.devices = {}  # mapping camera ids to opencv devices
        self.last_scan = None

    async def step(self):
        await super().step()
        await self.update_device_list()
        for uid, camera in self.world.usb_cameras.items():
            if not camera.capture:
                return
            bytes = await rosys.run.io_bound(self.capture_image, uid)
            camera.images.append(Image(camera_id=uid, data=bytes, time=self.world.time, size=camera.resolution))
        self.purge_old_images()

    def capture_image(self, id) -> bytes:
        _, image = self.devices[id].read()
        return cv2.imencode('.jpg', image)[1].tobytes()

    def purge_old_images(self):
        for camera in self.world.usb_cameras.values():
            while camera.images and camera.images[0].time < self.world.time - 5 * 60.0:
                del camera.images[0]

    async def update_device_list(self):
        if self.last_scan is not None and self.world.time < self.last_scan + 30:  # scan every 30 sec
            return
        self.last_scan = self.world.time
        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        for infos in output.split('\n\n'):
            if 'Cannot open device' in infos:
                continue
            match = re.search('\((.*)\)', infos)
            if match is None:
                continue
            uid = match.group(1)
            if uid not in self.world.usb_cameras:
                self.world.usb_cameras[uid] = UsbCamera(id=uid)
                self.log.info(f'adding camera {uid}')
                await event.call(event.Id.NEW_CAMERA, self.world.usb_cameras[uid])
            lines = infos.splitlines()
            num = int(lines[1].strip().lstrip('/dev/video'))
            if uid not in self.devices:
                device = self.get_capture_device(num)
                if device is None:
                    if uid in self.world.usb_cameras:
                        del self.world.usb_cameras[uid]
                else:
                    self.devices[uid] = device
            if uid in self.devices:
                await self.update_parameters(uid, num)

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

    async def update_parameters(self, uid: str, dev_num: int):
        output = await rosys.run.sh(['v4l2-ctl', '--all', '-d', str(dev_num)])
        size = re.search('Width/Height.*: (\d*)/(\d*)', output)
        self.world.usb_cameras[uid].resolution = ImageSize(width=int(size.group(1)), height=int(size.group(2)))
