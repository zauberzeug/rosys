import logging
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import numpy as np
from PIL import Image as PILImage

from .. import Camera, Image, ImageSize

log = logging.getLogger('rosys.replay_camera')


class ReplayCamera(Camera):
    def __init__(self, *,
                 camera_id: str,
                 images_dir: Path,
                 camera_name: str | None = None,
                 image_history_length=128) -> None:

        self.images_dir = images_dir
        self.previous_emitted_timestamp = -1.0
        self.images_by_timestamp: dict[float, Image] = {}
        self.timestamp_array = np.array([], dtype=float)

        super().__init__(id=camera_id,
                         name=camera_name,
                         connect_after_init=False,
                         base_path_overwrite=str(uuid4()),
                         image_history_length=image_history_length)

        self._load_images(images_dir)

    def _load_images(self, images_dir: Path) -> None:
        """Load images from the specified directory.

        The images should be named with their timestamp using to the format <yyyy-mm-dd_hh-mm-ss.ffffff>.jpg
        """

        for file in images_dir.iterdir():
            if file.suffix.lower() not in ['.jpg', '.jpeg', '.png']:
                continue

            try:
                timestamp = datetime.strptime(file.stem, '%Y-%m-%d_%H-%M-%S.%f').timestamp()
            except ValueError:
                log.warning('Skipping file %s, invalid timestamp', file)
                continue

            try:
                with open(file, 'rb') as f:
                    data = f.read()
                pil_image = PILImage.open(file)
                width, height = pil_image.size
                pil_image.close()
            except Exception as e:
                log.warning('Could not read image from file %s: %s', file, e)
                continue

            self.images_by_timestamp[timestamp] = Image(
                camera_id=self.id,
                size=ImageSize(width=width, height=height),
                time=timestamp,
                data=data
            )
        self.timestamp_array = np.array(list(self.images_by_timestamp.keys()), dtype=float)

    def step_to(self, timestamp: float) -> None:
        """Step to the image closest to the given timestamp."""
        if not self.images_by_timestamp:
            return

        closest_index = (np.abs(self.timestamp_array - timestamp)).argmin()
        closest_timestamp = self.timestamp_array[closest_index]
        if closest_timestamp == self.previous_emitted_timestamp:
            return  # No new image to emit

        self.previous_emitted_timestamp = closest_timestamp
        image = self.images_by_timestamp[closest_timestamp]

        self._add_image(image)

    @property
    def is_connected(self) -> bool:
        """To be interpreted as "ready to capture images"."""
        return True
