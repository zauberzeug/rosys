from dataclasses import dataclass
from typing import Optional

import numpy as np
import rosys

from ..world import BoxDetection, Image, PointDetection, World
from . import Detector


@dataclass
class SimulatedObject:
    category_name: str
    position: rosys.world.Point3d
    size: Optional[tuple[float]] = None


class DetectorSimulator(Detector):
    world: World
    noisy_image_points: bool = True

    def __init__(self) -> None:
        super().__init__()
        self.blocked_cameras: set = set()
        self.simulated_objects: list[SimulatedObject] = []
        self.uploaded = []

    @property
    def is_connected(self):
        return True

    async def detect(self, image: Image) -> tuple[Optional[BoxDetection], Optional[PointDetection]]:
        is_blocked = image.camera_id in self.blocked_cameras
        await rosys.sleep(0.4)
        image.detections = []
        if not is_blocked:
            self.add_detections(image)
        rosys.event.emit(rosys.event.Id.NEW_DETECTIONS, image)

    def add_detections(self, image: Image):
        camera = self.world.cameras[image.camera_id]
        for object in self.simulated_objects:
            image_point = camera.calibration_simulation.project_to_image(object.position)
            if not (0 <= image_point.x < image.size.width and 0 <= image_point.y < image.size.height):
                continue

            if object.size is None:
                detection = rosys.world.PointDetection(
                    category_name=object.category_name,
                    model_name='simulation',
                    confidence=1.0,
                    x=image_point.x,
                    y=image_point.y,
                )
            else:
                world_points = np.array([
                    [object.position.x + dx, object.position.y + dy, object.position.z + dz]
                    for dx in [-object.size[0] / 2, object.size[0] / 2]
                    for dy in [-object.size[1] / 2, object.size[1] / 2]
                    for dz in [-object.size[2] / 2, object.size[2] / 2]
                ])
                image_points = camera.calibration_simulation.project_array_to_image(world_points)
                detection = rosys.world.BoxDetection(
                    category_name=object.category_name,
                    model_name='simulation',
                    confidence=1.0,
                    x=image_points[:, 0].min(),
                    y=image_points[:, 1].min(),
                    width=image_points[:, 0].max() - image_points[:, 0].min(),
                    height=image_points[:, 1].max() - image_points[:, 1].min(),
                )

            if self.noisy_image_points:
                detection.x += np.random.randn()
                detection.y += np.random.randn()

            image.detections.append(detection)

    async def upload(self, image: rosys.world.Image):
        self.uploaded.append(image)