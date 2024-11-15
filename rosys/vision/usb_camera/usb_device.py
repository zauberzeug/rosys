from __future__ import annotations

import logging
import re
from collections.abc import Awaitable, Callable

import cv2
import numpy as np

from ... import rosys
from .usb_camera_scanner import device_nodes_from_uid

MJPG = cv2.VideoWriter.fourcc(*'MJPG')


def find_video_id(camera_uid: str) -> int | None:
    device_nodes = device_nodes_from_uid(camera_uid)
    video_ids = [
        int(node.strip().removeprefix('/dev/video'))
        for node in device_nodes
        if node.startswith('/dev/video')
    ]
    return min(video_ids) if video_ids else None


def to_bytes(image: np.ndarray) -> bytes:
    return image.tobytes()


class UsbDevice:

    def __init__(self, video_id: int, capture: cv2.VideoCapture, *,
                 on_new_image_data: Callable[[np.ndarray | bytes, float], Awaitable | None]) -> None:
        self._video_id: int = video_id
        self._capture: cv2.VideoCapture = capture
        self._on_new_image_data = on_new_image_data
        self._exposure_min: int = 0
        self._exposure_max: int = 0
        self._exposure_default: int = 0
        self._has_manual_exposure: bool = False
        self._video_formats: set[str] = set()
        self._image_is_jpg: bool = False

        self.set_video_format()

        self._capture_task = rosys.on_repeat(self._capture_image, interval=0.01)

    def __del__(self) -> None:
        self._capture.release()
        self._capture_task.stop()

    @property
    def video_formats(self) -> set[str]:
        return self._video_formats

    @property
    def image_is_jpg(self) -> bool:
        return self._image_is_jpg

    @staticmethod
    def from_uid(camera_id: str, on_new_image_data: Callable[[np.ndarray | bytes, float], Awaitable | None]) -> UsbDevice | None:
        video_id = find_video_id(camera_id)
        if video_id is None:
            logging.error('Could not find video device for camera %s', camera_id)
            return None

        capture = UsbDevice.create_capture(video_id)
        if capture is None:
            logging.error('Could not open video device %s', video_id)
            return None

        return UsbDevice(video_id=video_id, capture=capture, on_new_image_data=on_new_image_data)

    @staticmethod
    def create_capture(index: int) -> cv2.VideoCapture | None:
        capture = cv2.VideoCapture(index)
        if capture is None:
            return None
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    async def _capture_image(self) -> None:
        if not self._capture.isOpened():
            return
        read_result = await rosys.run.io_bound(self._capture.read)
        if read_result is None:
            return
        capture_success, frame = read_result

        if capture_success:
            timestamp = rosys.time()
            if self._image_is_jpg:
                bytes_ = await rosys.run.io_bound(to_bytes, frame[0])
                if bytes_ is None:
                    return
                result = self._on_new_image_data(bytes_, timestamp)
            else:
                result = self._on_new_image_data(frame, timestamp)
            if isinstance(result, Awaitable):
                await result

    async def release_capture(self) -> None:
        await rosys.run.io_bound(self._capture.release)

    async def load_value_ranges(self) -> None:
        output = await self.run_v4l('--all')
        if output is None:
            return
        match = re.search(r'exposure_absolute.*: min=(\d*).*max=(\d*).*default=(\d*).*', output)
        if match is not None:
            self._has_manual_exposure = True
            self._exposure_min = int(match.group(1))
            self._exposure_max = int(match.group(2))
            self._exposure_default = int(match.group(3))
        else:
            self._has_manual_exposure = False
        output = await self.run_v4l('--list-formats')
        matches = re.finditer(r"$.*'(.*)'.*", output)
        for m in matches:
            self._video_formats.add(m.group(1))

    async def run_v4l(self, *args) -> str:
        cmd = ['v4l2-ctl', '-d', str(self._video_id)]
        cmd.extend(args)
        return await rosys.run.sh(cmd)

    def set_video_format(self) -> None:
        if 'MJPG' in self._video_formats:
            # NOTE enforcing motion jpeg for now
            if self._capture.get(cv2.CAP_PROP_FOURCC) != MJPG:
                self._capture.set(cv2.CAP_PROP_FOURCC, MJPG)
            # NOTE disable video decoding (see https://stackoverflow.com/questions/62664621/read-jpeg-frame-from-mjpeg-self-without-decoding-in-python-opencv/70869738?noredirect=1#comment110818859_62664621)
            if self._capture.get(cv2.CAP_PROP_CONVERT_RGB) != 0:
                self._capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            # NOTE make sure there is no lag (see https://stackoverflow.com/a/30032945/364388)
            if self._capture.get(cv2.CAP_PROP_BUFFERSIZE) != 1:
                self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._image_is_jpg = True
        else:
            self._image_is_jpg = False

    def set_auto_exposure(self, auto: bool) -> None:
        if self._has_manual_exposure:
            if auto:
                self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            else:
                self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

    def set_exposure(self, value: float) -> None:
        if self._has_manual_exposure:
            is_auto_exposure = self.get_auto_exposure()
            if not is_auto_exposure:
                exposure = self._capture.get(cv2.CAP_PROP_EXPOSURE) / self._exposure_max
                if value != exposure:
                    self._capture.set(cv2.CAP_PROP_EXPOSURE, int(value * self._exposure_max))

    def get_auto_exposure(self) -> bool | None:
        return self._capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3

    def get_exposure(self) -> float | None:
        if not self._has_manual_exposure:
            return None
        return self._capture.get(cv2.CAP_PROP_EXPOSURE) / self._exposure_max

    def set_width(self, width: int) -> None:
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    def get_width(self) -> int:
        return int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    def set_height(self, height: int) -> None:
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_height(self) -> int:
        return int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def set_fps(self, fps: int) -> None:
        self._capture.set(cv2.CAP_PROP_FPS, fps)

    def get_fps(self) -> int:
        return int(self._capture.get(cv2.CAP_PROP_FPS))
