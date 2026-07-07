from __future__ import annotations

import asyncio
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
    MAX_READ_FAILURES = 10
    """Number of consecutive failed reads before the capture is considered lost (e.g. unplugged cable)."""

    def __init__(self, uid: str, video_id: int, capture: cv2.VideoCapture, *,
                 on_new_image_data: Callable[[np.ndarray | bytes, float], Awaitable | None],
                 reconnect_interval: float = 3.0) -> None:
        self.uid = uid
        self.log = logging.getLogger('rosys.vision.usb_camera.usb_device.' + uid)
        self._video_id: int = video_id
        self._capture: cv2.VideoCapture | None = capture
        self._on_new_image_data = on_new_image_data
        self.reconnect_interval = reconnect_interval
        self._exposure_min: int = 0
        self._exposure_max: int = 0
        self._exposure_default: int = 0
        self._has_manual_exposure: bool = False
        self._video_formats: set[str] = set()
        self._image_is_jpg: bool = False
        self._width: int | None = None
        self._height: int | None = None
        self._fps: int | None = None
        self._should_run: bool = True
        self._read_failures: int = 0
        self._capture_task: asyncio.Task | None = None

        self.set_video_format()

        self._start_capture_loop()

    def __del__(self) -> None:
        self._should_run = False
        if self._capture_task is not None and not self._capture_task.done():
            self._capture_task.cancel()
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    @property
    def is_connected(self) -> bool:
        """Whether a working capture is currently open (False while a lost capture is being reconnected)."""
        return self._capture is not None

    @property
    def is_active(self) -> bool:
        """Whether the self-healing capture loop is alive (capturing or waiting to reconnect)."""
        return self._capture_task is not None and not self._capture_task.done()

    @property
    def video_formats(self) -> set[str]:
        return self._video_formats

    @property
    def image_is_jpg(self) -> bool:
        return self._image_is_jpg

    @staticmethod
    def from_uid(camera_id: str, on_new_image_data: Callable[[np.ndarray | bytes, float], Awaitable | None],
                 reconnect_interval: float = 3.0) -> UsbDevice | None:
        video_id = find_video_id(camera_id)
        if video_id is None:
            logging.error('Could not find video device for camera %s', camera_id)
            return None

        capture = UsbDevice.create_capture(video_id)
        if capture is None:
            logging.error('Could not open video device %s', video_id)
            return None

        return UsbDevice(uid=camera_id, video_id=video_id, capture=capture,
                         on_new_image_data=on_new_image_data, reconnect_interval=reconnect_interval)

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

    def _start_capture_loop(self) -> None:
        if self._capture_task is not None and not self._capture_task.done():
            self.log.warning('[%s] capture loop already running', self.uid)
            return
        self._should_run = True
        self._capture_task = rosys.background_tasks.create(
            self._run_capture_loop(), name=f'usb capture {self.uid}')

    async def _run_capture_loop(self) -> None:
        """Keep a single capture session alive, reconnecting after `reconnect_interval` when it ends.

        Runs until `shutdown()` cancels the task.
        """
        try:
            while self._should_run:
                try:
                    await self._run_capture_session()
                except Exception:
                    self.log.exception('[%s] capture session failed', self.uid)
                if not self._should_run:
                    break
                self.log.info('[%s] capture ended; reconnecting in %.1f s', self.uid, self.reconnect_interval)
                await rosys.sleep(self.reconnect_interval)
        finally:
            if self._capture_task is asyncio.current_task():
                self._capture_task = None

    async def _run_capture_session(self) -> None:
        """Read frames until the capture is lost, then release it."""
        while self._should_run:
            if self._capture is None or not self._capture.isOpened():
                await self._open_capture()
                if self._capture is None:
                    return

            read_result = await rosys.run.io_bound(self._capture.read)
            if read_result is None:
                return
            capture_success, frame = read_result

            if not capture_success:
                self._read_failures += 1
                if self._read_failures >= self.MAX_READ_FAILURES:
                    self.log.warning('[%s] releasing capture after %d failed reads',
                                     self.uid, self._read_failures)
                    await self._release()
                    return
                await rosys.sleep(0.01)
                continue
            self._read_failures = 0

            timestamp = rosys.time()
            if self._image_is_jpg:
                bytes_ = await rosys.run.io_bound(to_bytes, frame[0])
                if bytes_ is None:
                    continue
                result = self._on_new_image_data(bytes_, timestamp)
            else:
                # convert bgr to rgb
                frame = frame[:, :, ::-1]
                result = self._on_new_image_data(frame, timestamp)
            if isinstance(result, Awaitable):
                await result

    async def _open_capture(self) -> None:
        """Re-find and re-open the capture for this camera."""
        await self._release()
        video_id = await rosys.run.io_bound(find_video_id, self.uid)
        if video_id is None:
            return
        capture = await rosys.run.io_bound(UsbDevice.create_capture, video_id)
        if capture is None:
            return
        self.log.info('[%s] connected on /dev/video%d', self.uid, video_id)
        self._video_id = video_id
        self._capture = capture
        self._read_failures = 0
        self.set_video_format()
        self._reapply_settings()

    async def _release(self) -> None:
        if self._capture is not None:
            await rosys.run.io_bound(self._capture.release)
            self._capture = None

    async def shutdown(self) -> None:
        self._should_run = False
        if self._capture_task is not None and not self._capture_task.done():
            self._capture_task.cancel()
            try:
                await asyncio.wait_for(self._capture_task, timeout=5)
            except TimeoutError:
                self.log.warning('[%s] timeout while waiting for capture task to cancel', self.uid)
            except asyncio.CancelledError:
                pass
            self._capture_task = None
        await self._release()

    def _reapply_settings(self) -> None:
        if self._width is not None:
            self.set_width(self._width)
        if self._height is not None:
            self.set_height(self._height)
        if self._fps is not None:
            self.set_fps(self._fps)

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
        if self._capture is None:
            return
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
        if self._capture is None:
            return
        if self._has_manual_exposure:
            if auto:
                self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            else:
                self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

    def set_exposure(self, value: float) -> None:
        if self._capture is None:
            return
        if self._has_manual_exposure:
            is_auto_exposure = self.get_auto_exposure()
            if not is_auto_exposure:
                exposure = self._capture.get(cv2.CAP_PROP_EXPOSURE) / self._exposure_max
                if value != exposure:
                    self._capture.set(cv2.CAP_PROP_EXPOSURE, int(value * self._exposure_max))

    def get_auto_exposure(self) -> bool | None:
        if self._capture is None:
            return None
        return self._capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3

    def get_exposure(self) -> float | None:
        if self._capture is None or not self._has_manual_exposure:
            return None
        return self._capture.get(cv2.CAP_PROP_EXPOSURE) / self._exposure_max

    def set_width(self, width: int) -> None:
        self._width = width
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    def get_width(self) -> int:
        if self._capture is None:
            return self._width or 0
        return int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    def set_height(self, height: int) -> None:
        self._height = height
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_height(self) -> int:
        if self._capture is None:
            return self._height or 0
        return int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def set_fps(self, fps: int) -> None:
        self._fps = fps
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FPS, fps)

    def get_fps(self) -> int:
        if self._capture is None:
            return self._fps or 0
        return int(self._capture.get(cv2.CAP_PROP_FPS))
