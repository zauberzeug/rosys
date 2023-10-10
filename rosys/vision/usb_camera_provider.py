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
        rosys.on_repeat(self.capture_images, 0.2)
        rosys.on_repeat(self.update_parameters, 1)
        rosys.on_repeat(self.update_device_list, 1)

        self.needs_backup: bool = False
        # persistence.register(self)

    @property
    def cameras(self) -> dict[str, UsbCamera]:
        return self._cameras

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self._cameras, UsbCamera, data.get('cameras', {}))

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
                await camera.deactivate()

    async def update_parameters(self) -> None:
        for camera in self._cameras.values():
            if camera.is_connected:
                await camera.set_parameters()

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        if output is None:
            self.log.error('Could not scan for USB cameras. Is video4linux (lib4vl) installed?')
            return
        output = '\n'.join([s for s in output.split('\n') if not s.startswith('Cannot open device')])
        for infos in output.split('\n\n'):
            match = re.search(r'\((.*)\)', infos)
            if match is None:
                continue
            uid = match.group(1)
            if uid not in self._cameras:
                self.add_camera(UsbCamera(id=uid))
            lines = infos.splitlines()
            if 'dev/video' not in lines[1]:
                continue
            if not self._cameras[uid].is_connected:
                await self._cameras[uid].activate()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.deactivate()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
