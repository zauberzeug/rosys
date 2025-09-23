import logging
from pathlib import Path

from ...rosys import Repeater
from ..camera_provider import CameraProvider
from .replay_camera import ReplayCamera

log = logging.getLogger('rosys.replay_camera_provider')


class ReplayCameraProvider(CameraProvider[ReplayCamera]):
    """This module collects and simulates cameras by looping over images in a drive."""

    def __init__(self, replay_folder: Path, replay_interval: float = .01) -> None:
        super().__init__()

        self._start_time: float = float('inf')
        self._current_time: float = 0.0
        self._end_time: float = 0.0

        self._playback_speed = 1.0

        self._find_cameras(replay_folder)
        self._set_time_interval()
        self._repeater = Repeater(self._step, interval=replay_interval)
        self._repeater.start()

    def pause(self) -> None:
        self._repeater.stop()

    def play(self) -> None:
        self._repeater.start()

    def stop(self) -> None:
        self._repeater.stop()
        self._current_time = self._start_time

    def set_speed(self, speed: float) -> None:
        """Set the playback speed.

        1.0 is real-time, 2.0 is double speed, 0.5 is half speed.
        You may use negative values to play backwards.
        """
        self._playback_speed = speed

    def jump_to(self, percent: float) -> None:
        """Jump to a specific point in the replay.

        :param percent: a value between 0.0 and 100.0, where 0.0 is the start and 100.0 is the end
        """
        assert 0.0 <= percent <= 100.0
        self._current_time = self._start_time + (percent / 100.0) * (self._end_time - self._start_time)

    def _find_cameras(self, replay_folder: Path) -> None:
        for file in replay_folder.iterdir():
            if file.is_dir():
                camera = ReplayCamera(camera_id=file.name,
                                      images_dir=file,
                                      camera_name=file.name,
                                      image_history_length=128)
                self.add_camera(camera)
                self.log.info('Added replay camera "%s" from folder "%s"', file.name, file)

    def _set_time_interval(self) -> None:
        for camera in self.cameras.values():
            if not camera.images_by_timestamp:
                continue

            timestamps = sorted(camera.images_by_timestamp.keys())
            self._start_time = min(self._start_time, timestamps[0])
            self._end_time = max(self._end_time, timestamps[-1])
            self._current_time = self._start_time

    async def _step(self) -> None:
        if self._current_time > self._end_time:
            self._current_time = self._start_time

        for camera in self.cameras.values():
            camera.step_to(self._current_time)

        self._current_time += self._repeater.interval * self._playback_speed

    async def update_device_list(self) -> None:
        pass
