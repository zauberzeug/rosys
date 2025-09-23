import logging
from datetime import datetime
from pathlib import Path

from .. import CameraProvider, Image

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
        self._record = False

    @property
    def record(self) -> bool:
        return self._record

    def set_record(self, value: bool):
        """Enable or disable recording images from the camera provider."""
        self._record = value
        if self._record:
            self.camera_provider.NEW_IMAGE.register(self._save_image)
        else:
            self.camera_provider.NEW_IMAGE.unregister(self._save_image)

    async def _save_image(self, image: Image):
        """Save the provided image to the data directory.

        Images are saved to a subfolder that is named with the camera id,
        with filenames <yyyy-mm-dd_hh-mm-ss.ffffff>.jpg

        :param image: the image to save
        """
        if not image.data:
            log.debug('No image data to save')
            return

        path = self.data_dir / image.camera_id.replace(':', '-')
        path.mkdir(parents=True, exist_ok=True)

        file_path = path / f'{datetime.fromtimestamp(image.time).strftime("%Y-%m-%d_%H-%M-%S.%f")}.jpg'

        with open(file_path, 'wb') as f:
            f.write(image.data)
            log.debug('Saved image from camera %s to %s', image.camera_id, file_path)
