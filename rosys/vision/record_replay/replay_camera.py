import logging
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import aiofiles
import numpy as np

from ... import run
from ..camera import Camera
from ..image import Image
from .constants import TIME_FORMAT

log = logging.getLogger('rosys.replay_camera')


class ReplayCamera(Camera):

    def __init__(self, *, camera_id: str, images_dir: Path, camera_name: str | None = None) -> None:
        self.images_dir = images_dir
        self.previous_emitted_index = -1
        self.image_paths: list[Path] = []
        self.timestamp_array = np.array([], dtype=float)

        super().__init__(id=camera_id, name=camera_name, connect_after_init=False, base_path_overwrite=str(uuid4()))

        self._load_image_paths(images_dir)

    def _load_image_paths(self, images_dir: Path) -> None:
        """Load images from the specified directory.

        The images should be named with their timestamp using to the format <yyyy-mm-dd_hh-mm-ss.ffffff>.jpg
        """
        path_timestamp_pairs = []
        for file in images_dir.iterdir():
            if file.suffix.lower() not in ['.jpg', '.jpeg', '.png']:
                continue

            try:
                timestamp = datetime.strptime(file.stem, TIME_FORMAT).timestamp()
            except ValueError:
                log.warning('Skipping file %s, invalid timestamp', file)
                continue
            path_timestamp_pairs.append((file, timestamp))

        path_timestamp_pairs.sort(key=lambda x: x[1])
        self.image_paths = [pair[0] for pair in path_timestamp_pairs]
        self.timestamp_array = np.array([pair[1] for pair in path_timestamp_pairs], dtype=float)

    def _find_closest_past_index(self, timestamp: float) -> int:
        return int(np.searchsorted(self.timestamp_array, timestamp, side='right') - 1)

    async def load_image_at_time(self, timestamp: float) -> None:
        """Step to the image closest to the given timestamp."""
        if not self.image_paths:
            return

        if timestamp < self.timestamp_array[0]:
            closest_past_index = 0
        elif timestamp > self.timestamp_array[-1]:
            closest_past_index = len(self.image_paths) - 1
        else:
            closest_past_index = self._find_closest_past_index(timestamp)
        if closest_past_index in [self.previous_emitted_index, -1]:
            return  # No new image to emit

        self.previous_emitted_index = closest_past_index
        image_path = self.image_paths[closest_past_index]

        async with aiofiles.open(image_path, 'rb') as f:
            data = await f.read()

        time = self.timestamp_array[closest_past_index]
        image = await run.cpu_bound(Image.from_jpeg_bytes, data, camera_id=self.id, time=time)
        if image is None:
            return
        self._add_image(image)

    @property
    def is_connected(self) -> bool:
        """To be interpreted as "ready to capture images"."""
        return True
