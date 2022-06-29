import io
import re
import shutil
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import PIL
import rosys
from numpy.typing import NDArray

from .. import event
from ..world import Image, ImageSize, UsbCamera
from .actor import Actor

MJPG = cv2.VideoWriter_fourcc(*'MJPG')
SCAN_INTERVAL = 10


@dataclass
class Device:
    '''device-specific infos are kept separately (world should not be aware of them)'''
    uid: str
    video_id: int
    capture: Optional[Any] = None  # cv2.VideoCapture device
    exposure_min: int = 0
    exposure_max: int = 0
    exposure_default: int = 0
    last_state: str = ''


def process_image(data: bytes, rotation: rosys.world.ImageRotation, crop: rosys.world.Rectangle = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x+crop.width), int(crop.y+crop.height)))
    if rotation == rosys.world.ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == rosys.world.ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == rosys.world.ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()


class UsbCameraCapture(Actor):
    interval: float = 0.3
    lag_reduction: int = 1

    def __init__(self):
        super().__init__()
        self.devices: dict[str, Device] = {}
        self.last_scan: Optional[float] = None

    async def step(self):
        await super().step()
        await self.update_device_list()
        for uid, camera in self.world.usb_cameras.items():
            if not camera.active:
                if uid in self.devices and self.devices[uid].capture is not None:
                    await self.deactivate(camera)
                continue
            try:
                if uid not in self.devices:
                    continue
                if self.devices[uid].capture is None:
                    await self.activate(uid)
                if self.devices[uid].capture is None:
                    self.log.warn(f'unexpected missing capture handle for {uid}')
                    continue
                elif self.devices[uid].last_state != camera.json():
                    await rosys.run.io_bound(self.set_parameters, camera, self.devices[uid])
                    self.devices[uid].last_state = camera.json()
                image = await rosys.run.io_bound(self.capture_image, uid)
                if image is None:
                    await self.deactivate(camera)
                    continue
                if camera.crop is None and \
                        (camera.rotation is None or camera.rotation == rosys.world.ImageRotation.NONE):
                    bytes = image[0].tobytes()
                else:
                    bytes = await rosys.run.cpu_bound(process_image, image[0].tobytes(), camera.rotation, camera.crop)
                size = camera.resolution or ImageSize(width=800, height=600)
                camera.images.append(Image(camera_id=uid, data=bytes, time=self.world.time, size=size))
            except:
                self.log.exception(f'could not capture image from {uid}')
                await self.deactivate(camera)
        self.purge_old_images()

    def capture_image(self, id) -> Any:
        _, image = self.devices[id].capture.read()
        return image

    async def activate(self, uid: str) -> None:
        camera = self.world.usb_cameras[uid]
        device = self.devices[uid]
        capture = await rosys.run.io_bound(self.get_capture_device, device.video_id)
        if capture is None:
            return
        self.devices[uid].capture = capture
        await rosys.run.io_bound(self.set_parameters, camera, device)
        self.log.info(f'activated {uid}')

    async def deactivate(self, camera: UsbCamera) -> None:
        if camera.id not in self.devices:
            return
        if self.devices[camera.id].capture is not None:
            self.log.info(f'deactivated {camera.id}')
            await rosys.run.io_bound(self.devices[camera.id].capture.release)
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
        if output is None:
            return
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
                self.devices[uid] = Device(uid=uid, video_id=num)
                await self.load_value_ranges(self.devices[uid])

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
            await self.deactivate(camera)
        self.devices.clear()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None

    def set_parameters(self, camera: UsbCamera, device: Device):
        camera.fps = int(device.capture.get(cv2.CAP_PROP_FPS))
        if camera.exposure is None:
            camera.exposure = device.exposure_default / device.exposure_max
        # NOTE enforcing motion jpeg for now
        if device.capture.get(cv2.CAP_PROP_FOURCC) != MJPG:
            device.capture.set(cv2.CAP_PROP_FOURCC, MJPG)
        # NOTE disable video decoding (see https://stackoverflow.com/questions/62664621/read-jpeg-frame-from-mjpeg-camera-without-decoding-in-python-opencv/70869738?noredirect=1#comment110818859_62664621)
        if device.capture.get(cv2.CAP_PROP_CONVERT_RGB) != 0:
            device.capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        # NOTE make sure there is no lag (see https://stackoverflow.com/a/30032945/364388)
        if device.capture.get(cv2.CAP_PROP_BUFFERSIZE) != 1:
            device.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
        if not camera.auto_exposure:
            exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
            if camera.exposure is not None and camera.exposure != exposure:
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
