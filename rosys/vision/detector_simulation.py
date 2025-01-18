from dataclasses import dataclass, field
from datetime import datetime
from typing import Literal
from uuid import uuid4

import numpy as np

from .. import rosys
from ..geometry import Point3d
from .calibratable_camera_provider import CalibratableCameraProvider
from .camera import CalibratableCamera
from .detections import BoxDetection, Detections, PointDetection
from .detector import Autoupload, Detector, DetectorInfo, ModelVersioningInfo
from .image import Image


@dataclass(slots=True, kw_only=True)
class SimulatedObject:
    category_name: str
    position: Point3d
    size: tuple[float, float, float] | None = None
    uuid: str = field(init=False)
    confidence: float = 1.0

    def __post_init__(self) -> None:
        self.uuid = str(uuid4())


class DetectorSimulation(Detector):
    """This detector simulates object detection.

    It requires a camera provider in order to check visibility using the cameras' calibrations.
    Individual camera IDs can be added to a set of `blocked_cameras` to simulate occlusions during pytests.
    A list of `simulated_objects` can be filled to define what can be detected.
    An optional `noise` parameter controls the spatial accuracy in pixels.
    An optional `detection_delay` parameter simulates the time it takes to process an image.
    """

    def __init__(self,
                 camera_provider: CalibratableCameraProvider, *,
                 noise: float = 1.0,
                 detection_delay: float = 0.4,
                 name: str | None = None,
                 ) -> None:
        super().__init__(name=name)

        self.camera_provider = camera_provider
        self.noise = noise

        self.blocked_cameras: set[str] = set()
        self.simulated_objects: list[SimulatedObject] = []
        self.detection_delay = detection_delay

    async def detect(self,
                     image: Image,
                     *,
                     autoupload: Autoupload = Autoupload.FILTERED,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None,
                     ) -> Detections | None:
        is_blocked = image.camera_id in self.blocked_cameras
        await rosys.sleep(self.detection_delay)
        image.set_detections(self.name, Detections())
        if not is_blocked:
            self.update_simulated_objects(image)
            self.detect_from_simulated_objects(image)
        self.NEW_DETECTIONS.emit(image)
        return image.get_detections(self.name)

    async def upload(self,
                     image: Image,
                     *,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None,
                     ) -> None:
        self.log.info('Uploading %s', image.id)

    async def fetch_detector_info(self) -> DetectorInfo:
        return DetectorInfo(operation_mode='simulation',
                            version_control='pause',
                            state=None,
                            categories=[])

    async def fetch_model_version_info(self) -> ModelVersioningInfo:
        return ModelVersioningInfo(current_version='None',
                                   target_version='None',
                                   loop_version='None',
                                   local_versions=[],
                                   version_control='pause')

    async def set_model_version(self, version: Literal['follow_loop', 'pause'] | str) -> None:
        self.log.info('Setting model version to %s', version)

    def update_simulated_objects(self, image: Image) -> None:
        pass

    def detect_from_simulated_objects(self, image: Image) -> None:
        if image.camera_id not in self.camera_provider.cameras:
            return
        camera = self.camera_provider.cameras[image.camera_id]
        assert isinstance(camera, CalibratableCamera)
        assert camera.calibration is not None
        detections = image.get_detections(self.name)
        assert detections is not None
        world_extrinsics = camera.calibration.extrinsics.resolve()
        for obj in self.simulated_objects:
            viewing_direction = np.array(world_extrinsics.rotation.R)[:, 2]
            object_direction = np.array(obj.position.tuple) - world_extrinsics.translation
            if np.dot(viewing_direction, object_direction) < 0:
                continue
            image_point = camera.calibration.project_to_image(obj.position)
            if image_point is None:
                continue
            if not (0 <= image_point.x < image.size.width and 0 <= image_point.y < image.size.height):
                continue

            if obj.size is None:
                detections.points.append(PointDetection(
                    category_name=obj.category_name,
                    model_name='simulation',
                    confidence=obj.confidence,
                    x=image_point.x + self.noise * np.random.randn(),
                    y=image_point.y + self.noise * np.random.randn(),
                    uuid=obj.uuid,
                ))
            else:
                world_points = np.array([
                    [obj.position.x + dx, obj.position.y + dy, obj.position.z + dz]
                    for dx in [-obj.size[0] / 2, obj.size[0] / 2]
                    for dy in [-obj.size[1] / 2, obj.size[1] / 2]
                    for dz in [-obj.size[2] / 2, obj.size[2] / 2]
                ])
                image_points = camera.calibration.project_to_image(world_points)
                if np.any(np.isnan(image_points)):
                    continue
                detections.boxes.append(BoxDetection(
                    category_name=obj.category_name,
                    model_name='simulation',
                    confidence=obj.confidence,
                    x=image_points[:, 0].min() + self.noise * np.random.randn(),
                    y=image_points[:, 1].min() + self.noise * np.random.randn(),
                    width=image_points[:, 0].max() - image_points[:, 0].min() + self.noise * np.random.randn(),
                    height=image_points[:, 1].max() - image_points[:, 1].min() + self.noise * np.random.randn(),
                    uuid=obj.uuid,
                ))
