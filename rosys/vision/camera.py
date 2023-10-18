from __future__ import annotations

import abc
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Generic, Optional, TypeVar
from uuid import uuid4

import numpy as np

from .. import persistence, rosys
from ..event import Event
from ..geometry import Rectangle, Rotation
from .calibration import Calibration, Extrinsics, Intrinsics
from .image import Image, ImageSize
from .image_rotation import ImageRotation
from .image_route import create_image_route

PARAMETER_TYPE = TypeVar('PARAMETER_TYPE')


class ConfigurableCameraMixin(abc.ABC):
    """A generalized interface for adjusting camera parameters like exposure, brightness or fps."""
    @dataclass(slots=True, kw_only=True)
    class ParameterInfo:
        name: str
        min: Optional[PARAMETER_TYPE] = None
        max: Optional[PARAMETER_TYPE] = None
        step: Optional[PARAMETER_TYPE] = None

        def __post_init__(self):
            if any([self.min, self.max, self.step]) and not all([self.min, self.max, self.step]):
                raise ValueError('min, max and step must be either all set or all None')

    @dataclass(slots=True, kw_only=True)
    class Parameter(Generic[PARAMETER_TYPE]):
        """A camera parameter which can be adjusted.
            Value ranges can either be a list of options or a min/max value with an optional step size.
        """
        info: ConfigurableCameraMixin.ParameterInfo
        value: Optional[PARAMETER_TYPE] = None
        setter: Callable
        getter: Callable

    _parameters: dict[str, Any] = {}

    def _register_parameter(self, name: str, getter: Callable, setter: Callable, default_value: Any,
                            min_value: Any = None, max_value: Any = None, step: Any = None) -> None:

        self._parameters[name] = self.Parameter(info=self.ParameterInfo(name=name, min=min_value, max=max_value, step=step),
                                                getter=getter, setter=setter, value=default_value)

    async def _apply_parameters(self, new_values: dict[str, Any]) -> None:
        for param in new_values:
            if param not in self._parameters:
                raise ValueError(f'Cannot set unknown parameter "{param}"')
            await self._parameters[param].setter(new_values[param])

    async def _apply_all_parameters(self) -> None:
        await self._apply_parameters({param: self._parameters[param].value for param in self._parameters})

    async def update_parameters(self, new_values: dict[str, Any]) -> None:
        for param in new_values:
            self._parameters[param].value = new_values[param]

        await self._apply_parameters(new_values)

    async def get_parameters(self, names: Optional[List[Any]]) -> dict[str, Any]:
        if names is None:
            names = self._parameters.keys()
        return {param: self._parameters[param].getter() for param in names}

    def get_capabilities(self) -> list[ParameterInfo]:
        return [param.info for param in self._parameters.values()]


class TransformCameraMixin(abc.ABC):
    crop: Optional[Rectangle] = None
    """region to crop on the original resolution before rotation"""

    rotation: ImageRotation = ImageRotation.NONE
    """rotation which should be applied after grabbing and cropping"""

    resolution: Optional[ImageSize] = None

    @property
    def image_resolution(self) -> Optional[ImageSize]:
        if self.resolution is None:
            return None
        width = int(self.crop.width) if self.crop else self.resolution.width
        height = int(self.crop.height) if self.crop else self.resolution.height
        if self.rotation in {ImageRotation.LEFT, ImageRotation.RIGHT}:
            width, height = height, width
        return ImageSize(width=width, height=height)

    @property
    def rotation_angle(self) -> int:
        """Rotation angle in degrees."""
        return int(self.rotation)

    @rotation_angle.setter
    def rotation_angle(self, value: int) -> None:
        self.rotation = ImageRotation.from_degrees(value)

    def rotate_clockwise(self) -> None:
        """Rotate the image clockwise by 90 degrees."""
        self.rotation_angle += 90

    def rotate_counter_clockwise(self) -> None:
        """Rotate the image counter clockwise by 90 degrees."""
        self.rotation_angle -= 90


class CalibratedCameraMixin(abc.ABC):
    calibration: Optional[Calibration] = None
    focal_length: Optional[float] = None

    @property
    def is_calibrated(self) -> bool:
        return self.calibration is not None

    def set_perfect_calibration(
        self,
        *,
        width=800, height=600,
        x: float = 0.0, y: float = 0.0, z: float = 1.0,
        roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
    ) -> None:
        self.calibration = Calibration(
            intrinsics=CalibratedCameraMixin.create_intrinsics(width, height),
            extrinsics=Extrinsics(rotation=Rotation.from_euler(roll, pitch, yaw), translation=[x, y, z]),
        )

    @staticmethod
    def create_intrinsics(width: int = 800, height: int = 600) -> Intrinsics:
        c = 570
        size = ImageSize(width=width, height=height)
        K: list[list[float]] = [[c, 0, size.width / 2], [0, c, size.height / 2], [0, 0, 1]]
        D: list[float] = [0] * 5
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)


@dataclass(slots=True, kw_only=True)
class Camera(abc.ABC):
    MAX_IMAGES = 256

    id: str
    images: deque[Image] = field(default_factory=lambda: deque(maxlen=Camera.MAX_IMAGES), metadata=persistence.exclude)
    name: Optional[str] = None

    connect_after_init: bool = True

    fps: Optional[int] = None
    """current frames per second (read only)"""

    streaming: bool = True

    _resolution: Optional[ImageSize] = None
    """physical resolution of the camera which should be used; camera may go into error state with wrong values"""

    NEW_IMAGE: Event = field(default_factory=Event, metadata=persistence.exclude)
    """a new image is available (argument: image)"""

    base_path: str = field(default_factory=lambda: f'images/{str(uuid4())}', metadata=persistence.exclude)

    def __post_init__(self) -> None:
        create_image_route(self)

        if self.name is None:
            self.name = self.id
        if self.connect_after_init:
            # start a new task to activate the camera
            rosys.on_startup(self.connect)

        async def stream() -> None:
            if self.streaming and self.is_connected:
                await self.capture_image()

        rosys.on_repeat(stream, interval=.1)

    def get_image_url(self, image: Image) -> str:
        return f'{self.base_path}/{image.time}'

    def get_latest_image_url(self) -> str:
        image = self.latest_captured_image
        if image is None:
            return f'{self.base_path}/placeholder'
        return self.get_image_url(image)

    @property
    def image_resolution(self) -> Optional[ImageSize]:
        return self._resolution

    @property
    def is_connected(self) -> bool:
        """To be interpreted as "ready to capture images"."""
        return False

    async def connect(self) -> None:
        # NOTE: due to a race condition, this method may be called multiple times even after any check for is_connected
        #       Make sure it is idempotent!
        pass

    async def disconnect(self) -> None:
        pass

    async def reconnect(self) -> None:
        await self.disconnect()
        await self.connect()

    @property
    def captured_images(self) -> list[Image]:
        return [i for i in self.images if i.data]

    @property
    def latest_captured_image(self) -> Optional[Image]:
        images = self.captured_images
        return images[-1] if images else None

    def get_recent_images(self, current_time: float, timespan: float = 10.0) -> list[Image]:
        return [i for i in self.captured_images if i.time > current_time - timespan]

    def _add_image(self, image: Image) -> None:
        self.images.append(image)
        self.NEW_IMAGE.emit(image)

    async def capture_image(self) -> None:
        raise NotImplementedError("Implement capture_image() in your camera class!")
