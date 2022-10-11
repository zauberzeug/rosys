import numpy as np
from nicegui import ui
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Cylinder, Group, Text, Texture

from .. import run
from .calibration import Calibration
from .camera import Camera
from .camera_projector import CameraProjector
from .camera_provider import CameraProvider


class CameraObjects(Group):
    '''This module provides a UI element for displaying cameras in a 3D scene.

    It requires a camera provider as a source of cameras as well as a camera projector to show the current images projected on the ground plane.
    The `px_per_m` argument can be used to scale the camera frustums.
    With `debug=True` camera IDs are shown (default: `False`).
    '''

    def __init__(self, camera_provider: CameraProvider, camera_projector: CameraProjector, *,
                 px_per_m: float = 10000, debug: bool = False) -> None:
        super().__init__()

        self.camera_provider = camera_provider
        self.camera_projector = camera_projector
        self.px_per_m = px_per_m
        self.debug = debug
        self.textures: dict[str, Texture] = {}

        ui.timer(1.0, self.update)

    @property
    def calibrated_cameras(self) -> dict[str, Camera]:
        return {id: camera for id, camera in self.camera_provider.cameras.items() if camera.calibration}

    def find_objects(self, type: str) -> dict[str, Object3D]:
        return {
            (obj.name or '').split('_', 1)[1]: obj
            for obj in self.view.objects.values()
            if (obj.name or '').split('_', 1)[0] == type
        }

    async def update(self) -> None:
        await self.update_cameras()
        await self.update_images()

    async def update_cameras(self) -> None:
        camera_groups = self.find_objects('camera')
        for uid, camera_group in camera_groups.items():
            if uid not in self.calibrated_cameras:
                camera_group.delete()
        for uid, camera in self.calibrated_cameras.items():
            if uid not in camera_groups:
                with Group().with_name(f'camera_{uid}') as camera_groups[uid]:
                    with Group() as pyramid:
                        Cylinder(0, np.sqrt(0.5), 1, 4) \
                            .rotate(-np.pi / 2, 0, np.pi / 4) \
                            .move(z=0.5) \
                            .material('#0088ff')
                        if self.debug:
                            Text(uid)
                    pyramid.scale(
                        camera.calibration.intrinsics.size.width / self.px_per_m,
                        camera.calibration.intrinsics.size.height / self.px_per_m,
                        camera.calibration.intrinsics.matrix[0][0] / self.px_per_m,
                    )
            camera_groups[uid].move(*camera.calibration.extrinsics.translation)
            camera_groups[uid].rotate_R(await run.cpu_bound(self.get_rotation, camera.calibration))

    @staticmethod
    def get_rotation(calibration: Calibration) -> list[list[float]]:
        return calibration.rotation.R  # computing euler rotation is cpu expensive

    async def update_images(self) -> None:
        newest_images = [camera.latest_captured_image for camera in self.calibrated_cameras.values()]
        newest_images = [i for i in newest_images if i is not None]

        current_uids = [image.camera_id for image in newest_images]
        for uid, texture in list(self.textures.items()):
            if uid not in current_uids:
                texture.delete()
                del self.textures[uid]

        z = 0
        for image in sorted(newest_images, key=lambda i: i.time):
            camera = self.calibrated_cameras.get(image.camera_id)
            if camera is None:
                continue
            projection = self.camera_projector.projections.get(camera.id)
            if projection is None:
                continue
            coordinates = [[point and [point[0], point[1], 0] for point in row] for row in projection.coordinates]

            url = self.camera_provider.get_image_url(image)
            if image.camera_id not in self.textures:
                self.textures[image.camera_id] = Texture(url, coordinates).with_name(f'image_{image.id}')
            texture = self.textures[image.camera_id]

            z += 0.001
            texture.move(z=z)
            if texture.args[0] != url:
                await texture.set_url(url)
            if not await run.cpu_bound(CameraProjector.allclose, texture.args[1], coordinates):
                await texture.set_coordinates(coordinates)
