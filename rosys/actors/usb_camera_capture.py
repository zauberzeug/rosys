import re
import shutil
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import rosys
from numpy.typing import NDArray

from .. import event
from ..world import Image, ImageSize, UsbCamera
from .actor import Actor

MJPG = cv2.VideoWriter_fourcc(*'MJPG')
SCAN_INTERVAL = 10


@dataclass
class Device:
    '''devices specific infos are kept seperately (world should not be aware of them)'''
    uid: str
    video_id: int
    capture: Any  # cv2.VideoCapture device
    exposure_min: int = 0
    exposure_max: int = 0
    exposure_default: int = 0


def process_image(image: NDArray, rotation: rosys.world.ImageRotation, crop: rosys.world.Rectangle = None) -> bytes:
    if crop is not None:
        image = image[int(crop.y):int(crop.y+crop.height), int(crop.x):int(crop.x+crop.width)]
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
                bytes = await rosys.run.cpu_bound(process_image, image, camera.rotation, camera.crop)
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
        if self.last_scan is not None and self.world.time < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = self.world.time
        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        for c in self.world.usb_cameras.values():
            c.connected = False
        output = '\n'.join([s for s in output.split('\n') if not s.startswith('Cannot open device')])
        for infos in output.split('\n\n'):
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
                    await self.load_value_ranges(self.devices[uid])
            if uid in self.devices:
                camera = self.world.usb_cameras[uid]
                device = self.devices[uid]
                camera.connected = True
                camera.fps = int(device.capture.get(cv2.CAP_PROP_FPS))
                if camera.exposure is None:
                    camera.exposure = device.exposure_default / device.exposure_max
                await rosys.run.io_bound(self.set_parameters, camera, device)

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

    def set_parameters(self, camera: UsbCamera, device: Device):
        # NOTE enforcing motion jpeg for now
        if device.capture.get(cv2.CAP_PROP_FOURCC) != MJPG:
            device.capture.set(cv2.CAP_PROP_FOURCC, MJPG)
        resolution = ImageSize(
            width=int(device.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(device.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        )
        if camera.resolution and camera.resolution != resolution:
            self.log.info(f'updating resolution of {camera.id} from {resolution} to {camera.resolution}')
            device.capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera.resolution.width)
            device.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera.resolution.height)
        auto_exposure = device.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3
        if camera.auto_exposure and not auto_exposure:
            self.log.info(f'activating auto-exposure for {camera.id}')
            device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # `v4l2-ctl -L` says "3: Aperture Priority Mode"
        if auto_exposure and not camera.auto_exposure:
            self.log.info(f'deactivating auto-exposure of {camera.id}')
            device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # `v4l2-ctl -L` says "1: Manual Mode"
        exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
        if camera.exposure != exposure:
            self.log.info(f'updating exposure of {camera.id} from {exposure} to {camera.exposure})')
            device.capture.set(cv2.CAP_PROP_EXPOSURE, int(camera.exposure * device.exposure_max))

    async def load_value_ranges(self, device: Device) -> None:
        output = await self.run_v4l(device, '--all')
        match = re.search('exposure_absolute.*: min=(\d*).*max=(\d*).*default=(\d*).*', output)
        device.exposure_min = int(match.group(1))
        device.exposure_max = int(match.group(2))
        device.exposure_default = int(match.group(3))

    async def run_v4l(self, device: Device, *args):
        cmd = ['v4l2-ctl', '-d', str(device.video_id)]
        cmd.extend(args)
        return await rosys.run.sh(cmd)
