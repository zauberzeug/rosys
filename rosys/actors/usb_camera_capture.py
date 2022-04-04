import re
import shutil
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import rosys

from .. import event
from ..world import Image, ImageSize, UsbCamera
from .actor import Actor


@dataclass
class Device:
    '''devices specific infos are kept seperately (world should not be aware of them)'''
    uid: str
    video_id: int
    capture: Any  # cv2.VideoCapture device
    resolution: Optional[ImageSize] = None
    exposure: Optional[float] = None


def process_image(image, rotation: rosys.world.ImageRotation) -> bytes:
    if rotation == rosys.world.ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == rosys.world.ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == rosys.world.ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    return cv2.imencode('.jpg', image)[1].tobytes()


class UsbCameraCapture(Actor):
    interval: float = 0.3

    def __init__(self):
        super().__init__()
        self.devices: dict[str, Device] = {}
        self.last_scan: Optional[float] = None

    async def step(self):
        await super().step()
        await self.update_device_list()
        for uid, camera in self.world.usb_cameras.items():
            if not camera.capture or not camera.connected:
                continue
            try:
                image = await rosys.run.io_bound(self.capture_image, uid)
                if image is None:
                    self.disconnect(uid)
                    continue
                bytes = await rosys.run.cpu_bound(process_image, image, camera.rotation)
                size = camera.resolution or ImageSize(width=800, height=600)
                camera.images.append(Image(camera_id=uid, data=bytes, time=self.world.time, size=size))
            except:
                self.log.exception(
                    f'could not capture image from {uid}; disconnecting device /dev/video{self.devices[uid].video_id}')
                self.disconnect(uid)
        self.purge_old_images()

    def capture_image(self, id) -> Any:
        capture = self.devices[id].capture
        _, image = capture.read()
        return image

    def disconnect(self, uid: str):
        self.log.info(f'disconnecting {uid}')
        camera = self.world.usb_cameras[uid]
        camera.connected = False
        self.devices[camera.id].capture.release()
        del self.devices[camera.id]

    def purge_old_images(self):
        for camera in self.world.usb_cameras.values():
            while camera.images and camera.images[0].time < self.world.time - 5 * 60.0:
                del camera.images[0]

    async def update_device_list(self):
        if self.last_scan is not None and self.world.time < self.last_scan + 30:  # scan every 30 sec
            return
        self.last_scan = self.world.time
        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        for c in self.world.usb_cameras.values():
            c.connected = False
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
            if 'dev/video' not in lines[1]:
                continue
            num = int(lines[1].strip().lstrip('/dev/video'))
            if uid not in self.devices:
                capture = self.get_capture_device(num)
                if capture is None:
                    if uid in self.world.usb_cameras:
                        self.world.usb_cameras[uid].connected = False
                else:
                    self.devices[uid] = Device(uid, num, capture)
            if uid in self.devices:
                self.world.usb_cameras[uid].connected = True
                await self.update_parameters(self.devices[uid])

    def get_capture_device(self, index: int):
        try:
            capture = cv2.VideoCapture(index)
            if capture is None:
                self.log.error(f'video device {index} is unavailable')
            elif not capture.isOpened():
                self.log.error(f'video device {index} can not be opened')
                capture.release()
            else:
                return capture
        except:
            self.log.exception(f'{index} device failed')

    async def tear_down(self):
        await super().tear_down()
        for camera in self.world.usb_cameras.values():
            camera.connected = False
        [device.capture.release() for device in self.devices.values()]
        self.devices.clear()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None

    async def update_parameters(self, device: Device):
        output = await self.run_v4l(device, '--all')
        size = re.search('Width/Height.*: (\d*)/(\d*)', output)
        device.resolution = ImageSize(width=int(size.group(1)), height=int(size.group(2)))
        camera = self.world.usb_cameras[device.uid]
        if camera.resolution and camera.resolution != device.resolution:
            self.log.info(f'updating resolution of {camera.id} from {device.resolution} to {camera.resolution}')
            await rosys.run.io_bound(UsbCameraCapture.update_resolution, device, camera.resolution)
            # TODO read exposure from output and update it correctly
            #     if device.exposure != camera.brightness:
            #         await self.update_exposure(device)

            # async def update_exposure(self, device: Device):
            #     exposure = self.world.usb_cameras[device.uid].exposure
            #     # TODO if expousre is None, set auto exposure
            #     await self.run_v4l(device, '--set-ctrl=brightness=0', '--set-ctrl=exposure_auto=1', f'--set-ctrl=exposure_absolute={exposure}')
            #     self.log.info(f'using new exposure {exposure}')

    @staticmethod
    def update_resolution(device: Device, size: ImageSize):
        device.capture.set(cv2.CAP_PROP_FRAME_WIDTH, size.width)
        device.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size.height)

    async def run_v4l(self, device: Device, *args):
        cmd = ['v4l2-ctl', '-d', str(device.video_id)]
        cmd.extend(args)
        return await rosys.run.sh(cmd)
