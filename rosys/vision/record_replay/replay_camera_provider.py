import logging
from pathlib import Path

from ... import rosys
from ..camera_provider import CameraProvider
from .replay_camera import ReplayCamera

log = logging.getLogger('rosys.replay_camera_provider')


class ReplayCameraProvider(CameraProvider[ReplayCamera]):
    """This module collects and simulates cameras by looping over images in a drive."""

    def __init__(self, replay_folder: Path, replay_interval: float = .01) -> None:
        super().__init__()

        self._start_time: float = float('inf')
        self._current_time: float = 0.0
        self._cameras_need_update = False
        self._end_time: float = 0.0

        self._playback_speed = 1.0

        self._running = True

        self._last_update_time: float = rosys.time()

        self._find_cameras(replay_folder)
        self._set_time_interval()

        self._repeater = rosys.on_repeat(self._step, interval=replay_interval)

    def pause(self) -> None:
        self._update_time()
        self._running = False

    def play(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False
        self.jump_to(0.0)

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
        self._set_replay_time(self._start_time + (percent / 100.0) * (self._end_time - self._start_time))

    def _set_replay_time(self, time: float) -> None:
        self._current_time = time
        self._cameras_need_update = True

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
            if not camera.timestamp_array:
                continue

            timestamps = camera.timestamp_array
            self._start_time = min(self._start_time, timestamps[0])
            self._end_time = max(self._end_time, timestamps[-1])
            self._current_time = self._start_time

    async def _step(self) -> None:
        print('step', flush=True)
        self._update_time()
        await self._update_camera_images()

    def _update_time(self) -> None:
        if not self._running:
            return

        now = rosys.time()
        new_replay_time = self._current_time + (now - self._last_update_time) * self._playback_speed
        self._set_replay_time(new_replay_time)
        if new_replay_time > self._end_time:
            new_replay_time = self._start_time + (new_replay_time - self._end_time)
        self._set_replay_time(new_replay_time)
        self._last_update_time = now

    async def _update_camera_images(self) -> None:
        if not self._cameras_need_update:
            return
        for camera in self.cameras.values():
            await camera.load_image_at_time(self._current_time)

    async def update_device_list(self) -> None:
        pass
