import logging
import re
from collections.abc import Awaitable, Callable

import cv2
import numpy as np

from ... import rosys
from ..capture_device import CaptureDevice
from .usb_camera_scanner import device_nodes_from_uid

MJPG = cv2.VideoWriter.fourcc(*'MJPG')


def find_device_node(camera_uid: str) -> str | None:
    """Find the lowest-numbered ``/dev/video*`` node of the camera with the given uid."""
    video_nodes = [node.strip() for node in device_nodes_from_uid(camera_uid) if node.startswith('/dev/video')]
    if not video_nodes:
        return None
    return min(video_nodes, key=lambda node: int(node.removeprefix('/dev/video')))


def to_bytes(image: np.ndarray) -> bytes:
    return image.tobytes()


class UsbDevice(CaptureDevice):
    MAX_READ_FAILURES = 10
    """Number of consecutive failed reads before the capture is considered lost (e.g. unplugged cable)."""

    def __init__(self, uid: str, *,
                 on_new_image_data: Callable[[np.ndarray | bytes, float], Awaitable | None],
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 reconnect_interval: float = 3.0) -> None:
        super().__init__(name=uid,
                         log=logging.getLogger('rosys.vision.usb_camera.usb_device.' + uid),
                         reconnect_interval=reconnect_interval)
        self.uid = uid
        self._device_node: str | None = None
        self._capture: cv2.VideoCapture | None = None
        self._on_new_image_data = on_new_image_data
        self._on_connect = on_connect
        self._exposure_min: int = 0
        self._exposure_max: int = 0
        self._exposure_default: int = 0
        self._has_manual_exposure: bool = False
        self._video_formats: set[str] = set()
        self._image_is_jpg: bool = False
        self._read_failures: int = 0
        self._unavailable_node: str | None = None

        self._start_capture_task()

    @property
    def is_connected(self) -> bool:
        """Whether a working capture is currently open (False while a lost capture is being reconnected)."""
        return self._capture is not None

    @property
    def video_formats(self) -> set[str]:
        return self._video_formats

    @property
    def image_is_jpg(self) -> bool:
        return self._image_is_jpg

    @staticmethod
    def create_capture(device_node: str) -> cv2.VideoCapture | None:
        capture = cv2.VideoCapture(device_node, cv2.CAP_V4L2)
        if capture is None:
            return None
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    async def _run_session(self) -> None:
        """Read frames until the capture is lost, then release it."""
        while self._keeps_running():
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
        """Find and open the capture for this camera; leaves ``_capture`` as ``None`` when it is unavailable."""
        await self._release()
        device_node = await rosys.run.io_bound(find_device_node, self.uid)
        if device_node is None:
            self.log.debug('[%s] no video device found', self.uid)
            return
        capture = await rosys.run.io_bound(UsbDevice.create_capture, device_node)
        if capture is None:
            if self._unavailable_node != device_node:
                self._unavailable_node = device_node
                self.log.warning('[%s] cannot open %s; another process may be using it', self.uid, device_node)
            return
        self._unavailable_node = None
        self.log.info('[%s] connected on %s', self.uid, device_node)
        self._device_node = device_node
        self._capture = capture
        self._read_failures = 0
        self.set_video_format()
        await self._invoke_on_connect()

    async def _invoke_on_connect(self) -> None:
        """Notify the owner that a capture session has been (re-)established, e.g. to reapply camera parameters."""
        if self._on_connect is None:
            return
        result = self._on_connect()
        if isinstance(result, Awaitable):
            await result

    async def _release(self) -> None:
        if self._capture is not None:
            await rosys.run.io_bound(self._capture.release)
            self._capture = None
        self._device_node = None

    async def _tear_down_session(self) -> None:
        await self._release()

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
        if self._device_node is None:
            return ''
        cmd = ['v4l2-ctl', '-d', self._device_node]
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
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    def get_width(self) -> int | None:
        if self._capture is None:
            return None
        return int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    def set_height(self, height: int) -> None:
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_height(self) -> int | None:
        if self._capture is None:
            return None
        return int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def set_fps(self, fps: int) -> None:
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FPS, fps)

    def get_fps(self) -> int | None:
        if self._capture is None:
            return None
        return int(self._capture.get(cv2.CAP_PROP_FPS))
