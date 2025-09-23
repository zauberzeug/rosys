import logging
from datetime import datetime
from pathlib import Path

import aiofiles

from ..camera_provider import CameraProvider
from ..image import Image
from .constants import TIME_FORMAT

log = logging.getLogger('rosys.image_recorder')


class ImageRecorder:
    """This module saves images from a CameraProvider to disk."""

    def __init__(self, camera_provider: CameraProvider, data_dir: Path) -> None:
        """This module saves images from a CameraProvider to disk.

        :param camera_provider: the camera provider to record images from
        :param data_dir: the directory to save images to. Images are saved in subdirectories
        """
        self.camera_provider = camera_provider
        self.data_dir = data_dir.expanduser()
        self._recording = False

    @property
    def recording(self) -> bool:
        return self._recording

    def set_recording(self, value: bool):
        """Enable or disable recording images from the camera provider."""
        self._recording = value
        if self._recording:
            self.camera_provider.NEW_IMAGE.register(self._save_image)
        else:
            self.camera_provider.NEW_IMAGE.unregister(self._save_image)

    async def _save_image(self, image: Image):
        """Save the provided image to the data directory.

        Images are saved to a subfolder named after the camera ID (with ':' replaced by '-'),
        using filenames in the format YYYY-MM-DD_HH-MM-SS.ffffff.jpg based on the image timestamp.

        :param image: the image to save
        """
        if not image.data:
            log.debug('No image data to save')
            return

        path = self.data_dir / image.camera_id.replace(':', '-')
        path.mkdir(parents=True, exist_ok=True)

        file_path = path / f'{datetime.fromtimestamp(image.time).strftime(TIME_FORMAT)}.jpg'

        async with aiofiles.open(file_path, 'wb') as f:
            await f.write(image.data)
            log.debug('Saved image from camera %s to %s', image.camera_id, file_path)
