from __future__ import annotations

import logging
from collections.abc import Awaitable, Callable

import numpy as np
from linuxpy.video.device import BufferType, Device, PixelFormat

from ... import rosys
from .usb_camera_scanner import device_nodes_from_uid


def find_video_id(camera_uid: str) -> int | None:
    device_nodes = device_nodes_from_uid(camera_uid)
    video_ids = [
        int(node.strip().removeprefix('/dev/video'))
        for node in device_nodes
        if node.startswith('/dev/video')
    ]
    return min(video_ids) if video_ids else None


class V4L2Device:

    def __init__(self, video_id: int, device: Device, *,
                 on_new_image_data: Callable[[np.ndarray | bytes, float], Awaitable | None]) -> None:
        self._video_id: int = video_id
        self._device: Device = device
        self._on_new_image_data = on_new_image_data
        self._exposure_min: int = 0
        self._exposure_max: int = 0
        self._exposure_default: int = 0
        self._has_manual_exposure: bool = False
        self._video_formats: set[str] = set()
        self._image_is_jpg: bool = False
        self.log = logging.getLogger('rosys.vision.usb_device')
        rosys.background_tasks.create(self._capture_images())

    def __del__(self) -> None:

        self.close()

    @property
    def video_formats(self) -> set[str]:
        return self._video_formats

    @property
    def image_is_jpg(self) -> bool:
        return self._image_is_jpg

    @staticmethod
    async def from_uid(camera_id: str, on_new_image_data: Callable[[np.ndarray | bytes, float], Awaitable | None]) -> V4L2Device | None:
        video_id = find_video_id(camera_id)
        if video_id is None:
            logging.error('Could not find video device for camera %s', camera_id)
            return None

        try:
            device = Device.from_id(video_id)
            device.open()
            try:
                _ = device.fileno()
            except Exception:
                logging.error('Could not open video device %s', video_id)
                return None

            usb_device = V4L2Device(video_id=video_id, device=device, on_new_image_data=on_new_image_data)
            await usb_device.load_value_ranges()
            # await rosys.sleep(1)
            usb_device.set_video_format()
            return usb_device
        except Exception as e:
            logging.error('Error initializing video device %s: %s', video_id, e)
            return None

    async def _capture_images(self) -> None:
        """Capture and process frames."""
        try:
            async for frame in self._device:
                timestamp = rosys.time()
                if self._image_is_jpg:
                    result = self._on_new_image_data(frame.data, timestamp)
                else:
                    # TODO: Convert frame.data to numpy array
                    result = self._on_new_image_data(frame.data, timestamp)
                if isinstance(result, Awaitable):
                    await result
        except Exception:
            self.log.exception('Error capturing frame for video device %s', self._video_id)

    def close(self) -> None:
        if self._device:
            try:
                self._device.close()
            except Exception:
                self.log.exception('Error closing video device %s', self._video_id)

    async def load_value_ranges(self) -> None:
        try:
            controls = self._device.controls
            if hasattr(controls, 'exposure_absolute'):
                ctrl = controls.exposure_absolute
                self._has_manual_exposure = True
                self._exposure_min = ctrl.minimum
                self._exposure_max = ctrl.maximum
                self._exposure_default = ctrl.default
            else:
                self._has_manual_exposure = False

            for fmt in self._device.info.formats:
                self._video_formats.add(fmt.pixel_format.name)
        except Exception:
            self.log.exception('Error loading device information')

    def set_video_format(self) -> None:
        """Try to set MJPEG with max resolution."""
        try:
            if 'MJPEG' in [format.pixel_format.name for format in self._device.info.formats]:
                sizes = self._device.info.frame_sizes
                max_size = max(sizes, key=lambda x: x.width * x.height)
                self._device.set_format(
                    width=max_size.width,
                    height=max_size.height,
                    pixel_format=PixelFormat.MJPEG,
                    buffer_type=BufferType.VIDEO_CAPTURE
                )
                self._image_is_jpg = True
            else:
                self._image_is_jpg = False

            self._device.buffer_count = 1
        except Exception:
            self.log.exception('Error setting video format')

    def set_auto_exposure(self, auto: bool) -> None:
        try:
            if self._has_manual_exposure:
                controls = self._device.controls
                if hasattr(controls, 'exposure_auto'):
                    ctrl = controls.exposure_auto
                    if hasattr(ctrl, 'value'):
                        if isinstance(ctrl.value, bool):
                            ctrl.value = auto
                        else:
                            ctrl.value = 3 if auto else 1  # 3=auto, 1=manual
        except Exception:
            self.log.exception('Error setting auto exposure')

    def set_exposure(self, value: float) -> None:
        try:
            if self._has_manual_exposure and not self.get_auto_exposure():
                controls = self._device.controls
                if hasattr(controls, 'exposure_absolute'):
                    ctrl = controls.exposure_absolute
                    new_value = int(value * (self._exposure_max - self._exposure_min) + self._exposure_min)
                    if new_value != ctrl.value:
                        ctrl.value = new_value
        except Exception:
            self.log.exception('Error setting exposure')

    def get_auto_exposure(self) -> bool | None:
        try:
            controls = self._device.controls
            if hasattr(controls, 'exposure_auto'):
                ctrl = controls.exposure_auto
                if isinstance(ctrl.value, bool):
                    return ctrl.value
                else:
                    return ctrl.value == 3  # 3=auto, 1=manual
            return None
        except Exception:
            return None

    def get_exposure(self) -> float | None:
        try:
            if not self._has_manual_exposure:
                return None
            controls = self._device.controls
            if hasattr(controls, 'exposure_absolute'):
                ctrl = controls.exposure_absolute
                return (ctrl.value - self._exposure_min) / (self._exposure_max - self._exposure_min)
            return None
        except Exception:
            return None

    def set_width(self, width: int) -> None:
        return
        try:
            fmt = self._device.get_format(BufferType.VIDEO_CAPTURE)
            current_height = fmt.height

            if 'MJPEG' in self._video_formats:
                pixel_format = PixelFormat.MJPEG
            else:
                # Use whatever format is currently set
                pixel_format = fmt.pixel_format if hasattr(fmt, 'pixel_format') else None

            if pixel_format:
                self._device.set_format(
                    width=width,
                    height=current_height,
                    pixel_format=pixel_format,
                    buffer_type=BufferType.VIDEO_CAPTURE
                )
        except Exception as e:
            self.log.error(f'Error setting width: {e}')

    def get_width(self) -> int:
        try:
            fmt = self._device.get_format(BufferType.VIDEO_CAPTURE)
            return fmt.width
        except Exception:
            return 0

    def set_height(self, height: int) -> None:
        return
        try:
            fmt = self._device.get_format(BufferType.VIDEO_CAPTURE)
            current_width = fmt.width

            if 'MJPEG' in self._video_formats:
                pixel_format = PixelFormat.MJPEG
            else:
                # Use whatever format is currently set
                pixel_format = fmt.pixel_format if hasattr(fmt, 'pixel_format') else None

            if pixel_format:
                self._device.set_format(
                    width=current_width,
                    height=height,
                    pixel_format=pixel_format,
                    buffer_type=BufferType.VIDEO_CAPTURE
                )
        except Exception as e:
            self.log.error(f'Error setting height: {e}')

    def get_height(self) -> int:
        try:
            fmt = self._device.get_format(BufferType.VIDEO_CAPTURE)
            return fmt.height
        except Exception:
            return 0

    def set_fps(self, fps: int) -> None:
        return
        try:
            self._device.set_fps(fps)
        except Exception as e:
            self.log.error(f'Error setting FPS: {e}')

    def get_fps(self) -> int:
        try:
            return self._device.get_fps()
        except Exception:
            return 0
