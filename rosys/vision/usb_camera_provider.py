import logging
import re
import shutil
from typing import Any, Optional

from .. import persistence, rosys
from .camera_provider import CameraProvider
from .usb_camera import UsbCamera

SCAN_INTERVAL = 10


class UsbCameraProvider(CameraProvider):
    """This module collects and provides real USB cameras.

    Camera devices are discovered through video4linux (v4l) and accessed with openCV.
    Therefore the program v4l2ctl and openCV (including python bindings) must be available.
    """

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.usb_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, UsbCamera] = {}

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_parameters, 1)
        rosys.on_repeat(self.update_device_list, 1)

        self.needs_backup: bool = False
        # persistence.register(self)

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, dict]) -> None:
        persistence.replace_dict(self._cameras, UsbCamera, data.get('cameras', {}))
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    async def capture_images(self) -> None:
        for uid, camera in self._cameras.items():
            try:
                if not camera.is_connected:
                    continue

                image = await camera.capture_image()
                if image is None:
                    raise Exception(f'could not capture image from {uid}')

                self.add_image(camera, image)
            except Exception:
                self.log.exception(f'could not capture image from {uid}')
                await camera.disconnect()

    async def update_parameters(self) -> None:
        for camera in self._cameras.values():
            if camera.is_connected:
                await camera.set_parameters()

    @staticmethod
    async def scan_for_cameras() -> list[UsbCamera]:
        uids = []

        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        if output is None:
            raise Exception('Could not scan for USB cameras. Is video4linux (lib4vl) installed?')
        output = '\n'.join([s for s in output.split('\n') if not s.startswith('Cannot open device')])
        for infos in output.split('\n\n'):
            match = re.search(r'\((.*)\)', infos)
            if match is None:
                continue
            uids.append(match.group(1))

        return uids

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        camera_uids = await UsbCameraProvider.scan_for_cameras()
        for uid in camera_uids:
            if not uid in self._cameras:
                self.add_camera(UsbCamera(id=uid))

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
