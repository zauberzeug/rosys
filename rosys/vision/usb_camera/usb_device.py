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


class UsbDevice:

    def __init__(self, video_id: int, capture: cv2.VideoCapture, *,
                 on_new_image_data: Callable[[np.ndarray], Awaitable | None]) -> None:
        self.video_id: int = video_id
        self.capture: cv2.VideoCapture = capture
        self.on_new_image_data: Callable[[np.ndarray], Awaitable | None] = on_new_image_data
        self.exposure_min: int = 0
        self.exposure_max: int = 0
        self.exposure_default: int = 0
        self.has_manual_exposure: bool = False
        self.video_formats: set[str] = set()

        self.set_video_format()

        self.capture_task = rosys.on_repeat(self._capture_image, interval=0.01)

    def __del__(self) -> None:
        self.capture.release()
        self.capture_task.stop()

    @staticmethod
    def from_uid(camera_id: str, on_new_image_data: Callable[[np.ndarray], Awaitable | None]) -> UsbDevice | None:
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
        if not self.capture.isOpened():
            return
        result = await rosys.run.io_bound(self.capture.read)
        if result is None:
            return
        capture_success, frame = result
        if capture_success:
            result = self.on_new_image_data(frame)
            if isinstance(result, Awaitable):
                await result

    async def load_value_ranges(self) -> None:
        output = await self.run_v4l('--all')
        if output is None:
            return
        match = re.search(r'exposure_absolute.*: min=(\d*).*max=(\d*).*default=(\d*).*', output)
        if match is not None:
            self.has_manual_exposure = True
            self.exposure_min = int(match.group(1))
            self.exposure_max = int(match.group(2))
            self.exposure_default = int(match.group(3))
        else:
            self.has_manual_exposure = False
        output = await self.run_v4l('--list-formats')
        matches = re.finditer(r"$.*'(.*)'.*", output)
        for m in matches:
            self.video_formats.add(m.group(1))

    async def run_v4l(self, *args) -> str:
        cmd = ['v4l2-ctl', '-d', str(self.video_id)]
        cmd.extend(args)
        return await rosys.run.sh(cmd)

    def set_video_format(self) -> None:
        if 'MJPG' in self.video_formats:
            # NOTE enforcing motion jpeg for now
            if self.capture.get(cv2.CAP_PROP_FOURCC) != MJPG:
                self.capture.set(cv2.CAP_PROP_FOURCC, MJPG)
            # NOTE disable video decoding (see https://stackoverflow.com/questions/62664621/read-jpeg-frame-from-mjpeg-self-without-decoding-in-python-opencv/70869738?noredirect=1#comment110818859_62664621)
            if self.capture.get(cv2.CAP_PROP_CONVERT_RGB) != 0:
                self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            # NOTE make sure there is no lag (see https://stackoverflow.com/a/30032945/364388)
            if self.capture.get(cv2.CAP_PROP_BUFFERSIZE) != 1:
                self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
