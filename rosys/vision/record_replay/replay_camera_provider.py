import logging
from datetime import datetime
from pathlib import Path

from nicegui import ui

from ... import rosys
from ..camera_provider import CameraProvider
from .replay_camera import ReplayCamera

log = logging.getLogger('rosys.replay_camera_provider')


class ReplayCameraProvider(CameraProvider[ReplayCamera]):
    """This module collects and simulates cameras by looping over images in a drive.

    NOTE: The first image of each camera will be emitted at any time before the timestamp of the second image.
    Therefore, setting the time to 0.0 will cause all cameras to emit their first image.

    :param replay_folder: path to the root replay folder containing the camera folders
    :param replay_interval: the interval at which the provider checks for new images (in seconds)
    """

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
        self._last_update_time = rosys.time()
        self._running = True

    def stop(self) -> None:
        self._running = False
        self.jump_to(0.0)

    def toggle_running(self) -> None:
        if self._running:
            self.pause()
        else:
            self.play()

    def create_ui(self, *, skip_time: float = 2.0, time_label_format: str = '%d.%m.%Y %H:%M:%S') -> None:
        """Create a simple UI for controlling the replay.

        :param skip_time: the amount of time to skip when pressing the skip buttons (in seconds)
        :param time_label_format: the format to be used for the current time label (used by ``datetime.strftime``)
        """
        with ui.row(align_items='center').classes('w-full gap-2'):
            ui.button(icon='sym_o_replay') \
                .on_click(lambda: self.jump_to_time(max(self._current_time - skip_time, self._start_time))) \
                .props('dense size=md') \
                .tooltip(f'<- {skip_time}s')
            ui.button(icon='play_arrow', on_click=self.toggle_running) \
                .bind_icon_from(self, '_running', backward=lambda p: 'pause' if p else 'play_arrow') \
                .props('dense size=md')
            ui.button(icon='sym_o_forward_media') \
                .on_click(lambda: self.jump_to_time(min(self._current_time + skip_time, self._end_time))) \
                .props('dense size=md') \
                .tooltip(f'-> {skip_time}s')
            ui.slider(min=0.25, max=4, step=0.25, on_change=lambda e: self.set_speed(e.value)) \
                .bind_value_from(self, '_playback_speed') \
                .props('label snap') \
                .classes('w-32 ml-4')
        s = ui.slider(min=self._start_time, max=self._end_time) \
            .bind_value(self, '_current_time') \
            .on('change', lambda e: self.jump_to_time(e.args)) \
            .props('label-always') \
            .classes('w-full')
        s.bind_value_to(s.props, 'label-value', lambda x: f'{datetime.fromtimestamp(x).strftime(time_label_format)}')

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
        assert 0.0 <= percent <= 100.0, 'Percent must be between 0.0 and 100.0'
        self.jump_to_time(self._start_time + (percent / 100.0) * (self._end_time - self._start_time))

    def jump_to_time(self, time: float) -> None:
        """Jump to a specific time in the replay.

        :param time: time in seconds
        """
        assert self._start_time <= time <= self._end_time, f'Time must be between {self._start_time} & {self._end_time}'
        self.clear_camera_images()
        self._set_replay_time(time)

    def _set_replay_time(self, time: float) -> None:
        self._current_time = time
        self._cameras_need_update = True

    def _find_cameras(self, replay_folder: Path) -> None:
        for file in replay_folder.iterdir():
            if file.is_dir():
                cam_name = file.name.replace('--', ':')
                camera = ReplayCamera(camera_id=cam_name, images_dir=file, camera_name=cam_name)
                self.add_camera(camera)
                self.log.info('Added replay camera "%s" from folder "%s"', cam_name, file)

    def _set_time_interval(self) -> None:
        for camera in self.cameras.values():
            if not camera.image_paths:
                continue

            timestamps = camera.timestamp_array
            self._start_time = min(self._start_time, timestamps[0])
            self._end_time = max(self._end_time, timestamps[-1])
            self._current_time = self._start_time

    async def _step(self) -> None:
        self._update_time()
        await self._update_camera_images()

    def _update_time(self) -> None:
        if not self._running:
            return

        now = rosys.time()
        new_replay_time = self._current_time + (now - self._last_update_time) * self._playback_speed
        if new_replay_time > self._end_time:
            new_replay_time = self._start_time + (new_replay_time - self._end_time)
        self._set_replay_time(new_replay_time)
        self._last_update_time = now

    async def _update_camera_images(self) -> None:
        if not self._cameras_need_update:
            return
        for camera in self.cameras.values():
            await camera.load_image_at_time(self._current_time)

    def clear_camera_images(self) -> None:
        for camera in self.cameras.values():
            camera.images.clear()

    async def update_device_list(self) -> None:
        pass
