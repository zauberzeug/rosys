import io
import logging
import re
import shutil
from dataclasses import dataclass, field
from typing import Any, Optional

import cv2
import numpy as np
import PIL
import rosys

from .. import persistence
from ..geometry import Rectangle
from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .usb_camera import ImageRotation, UsbCamera

MJPG = cv2.VideoWriter_fourcc(*'MJPG')
SCAN_INTERVAL = 10


@dataclass(slots=True, kw_only=True)
class Device:
    uid: str
    video_id: int
    capture: Optional[Any] = None  # cv2.VideoCapture device
    exposure_min: int = 0
    exposure_max: int = 0
    exposure_default: int = 0
    last_state: dict = field(default_factory=dict)
    video_formats: set[str] = field(default_factory=set)


def process_jpeg_image(data: bytes, rotation: ImageRotation, crop: Rectangle = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x+crop.width), int(crop.y+crop.height)))
    if rotation == ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()


def process_ndarray_image(image: np.ndarray, rotation: ImageRotation, crop: Rectangle = None) -> bytes:
    if crop is not None:
        image = image[int(crop.y):int(crop.y+crop.height), int(crop.x):int(crop.x+crop.width)]
    if rotation == ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    return cv2.imencode('.jpg', image)[1].tobytes()


def to_bytes(image: Any) -> bytes:
    return image[0].tobytes()


class UsbCameraProviderHardware(CameraProvider):
    '''This module collects and provides real USB cameras.

    Camera devices are discovered through video4linux (v4l) and accessed with openCV.
    Therefore the program v4l2ctl and openCV (including python bindings) must be available.
    '''

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.usb_camera_provider')

        self.devices: dict[str, Device] = {}
        self.last_scan: Optional[float] = None
        self._cameras: dict[str, UsbCamera] = {}

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.capture_images, 0.2)
        rosys.on_repeat(self.update_parameters, 1)
        rosys.on_repeat(self.update_device_list, 1)
        rosys.on_repeat(lambda: self.prune_images(max_age_seconds=1.0), 5.0)

        self.needs_backup: bool = False
        persistence.register(self)

    @property
    def cameras(self) -> dict[str, UsbCamera]:
        return self._cameras

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self._cameras, UsbCamera, data.get('cameras', {}))

    async def capture_images(self) -> None:
        for uid, camera in self._cameras.items():
            if not camera.active:
                if uid in self.devices and self.devices[uid].capture is not None:
                    await self.deactivate(camera)
                continue
            try:
                if uid not in self.devices:
                    continue
                device = self.devices[uid]
                if device.capture is None:
                    await self.activate(uid)
                if device.capture is None:
                    self.log.warn(f'unexpected missing capture handle for {uid}')
                    continue
                image = await rosys.run.io_bound(self.capture_image, uid)
                if image is None:
                    await self.deactivate(camera)
                    continue
                if 'MJPG' in device.video_formats:
                    bytes = await rosys.run.io_bound(to_bytes, image)
                    if camera.crop or camera.rotation != ImageRotation.NONE:
                        bytes = await rosys.run.cpu_bound(process_jpeg_image, bytes, camera.rotation, camera.crop)
                else:
                    bytes = await rosys.run.cpu_bound(process_ndarray_image, image, camera.rotation, camera.crop)
                size = camera.resolution or ImageSize(width=800, height=600)
                camera.images.append(Image(camera_id=uid, data=bytes, time=rosys.time(), size=size))
            except:
                self.log.exception(f'could not capture image from {uid}')
                await self.deactivate(camera)

    async def update_parameters(self) -> None:
        for uid, camera in self._cameras.items():
            if camera.active and uid in self.devices:
                continue
            await rosys.run.io_bound(self.set_parameters, camera, self.devices[uid])

    def capture_image(self, id) -> Any:
        _, image = self.devices[id].capture.read()
        return image

    async def activate(self, uid: str) -> None:
        camera = self._cameras[uid]
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

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        if output is None:
            return
        output = '\n'.join([s for s in output.split('\n') if not s.startswith('Cannot open device')])
        for infos in output.split('\n\n'):
            match = re.search(r'\((.*)\)', infos)
            if match is None:
                continue
            uid = match.group(1)
            if uid not in self._cameras:
                self._cameras[uid] = UsbCamera(id=uid)
                self.log.info(f'adding camera {uid}')
                await self.CAMERA_ADDED.call(self._cameras[uid])
            lines = infos.splitlines()
            if 'dev/video' not in lines[1]:
                continue
            num = int(lines[1].strip().lstrip('/dev/video'))
            if uid not in self.devices:
                self.devices[uid] = Device(uid=uid, video_id=num)
                await self.load_value_ranges(self.devices[uid])

    def get_capture_device(self, index: int) -> None:
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

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await self.deactivate(camera)
        self.devices.clear()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None

    def set_parameters(self, camera: UsbCamera, device: Device) -> None:
        camera.fps = int(device.capture.get(cv2.CAP_PROP_FPS))
        if not camera.auto_exposure and camera.exposure is None and device.exposure_max > 0:
            camera.exposure = device.exposure_default / device.exposure_max
        if 'MJPG' in device.video_formats:
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
        if not camera.auto_exposure and device.exposure_max > 0:
            exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
            if camera.exposure is not None and camera.exposure != exposure:
                self.log.info(f'updating exposure of {camera.id} from {exposure} to {camera.exposure})')
                device.capture.set(cv2.CAP_PROP_EXPOSURE, int(camera.exposure * device.exposure_max))

    async def load_value_ranges(self, device: Device) -> None:
        output = await self.run_v4l(device, '--all')
        if output is None:
            return
        match = re.search(r'exposure_absolute.*: min=(\d*).*max=(\d*).*default=(\d*).*', output)
        if match is not None:
            device.exposure_min = int(match.group(1))
            device.exposure_max = int(match.group(2))
            device.exposure_default = int(match.group(3))
        output = await self.run_v4l(device, '--list-formats')
        match = re.finditer(r"$.*'(.*)'.*", output)
        for m in match:
            device.video_formats.add(m.group(1))

    async def run_v4l(self, device: Device, *args) -> None:
        cmd = ['v4l2-ctl', '-d', str(device.video_id)]
        cmd.extend(args)
        return await rosys.run.sh(cmd)
