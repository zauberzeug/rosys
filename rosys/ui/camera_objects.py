import numpy as np
from nicegui import ui
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Cylinder, Group, Texture
from rosys.actors import CameraProjector
from rosys.world import Camera

from ..world import World


class CameraObjects(Object3D):
    world: World = None

    def __init__(self, *, px_per_m: float = 10000):
        super().__init__('group')
        self.px_per_m = px_per_m
        self.textures: dict[str, Texture] = {}
        ui.timer(1.0, self.update)

    @property
    def calibrated_cameras(self) -> dict[str, Camera]:
        return {
            id: camera
            for id, camera in self.world.cameras.items()
            if camera.calibration is not None and camera.calibration.is_complete
        }

    def find_objects(self, type: str) -> dict[str, Object3D]:
        return {
            (obj.name or '').split('_', 1)[1]: obj
            for obj in self.view.objects.values()
            if (obj.name or '').split('_', 1)[0] == type
        }

    async def update(self) -> bool:
        self.update_cameras()
        await self.update_images(None)
        return False  # NOTE: avoid JustPy page_update

    def update_cameras(self) -> bool:
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
                    pyramid.scale(
                        camera.calibration.intrinsics.size.width / self.px_per_m,
                        camera.calibration.intrinsics.size.height / self.px_per_m,
                        camera.calibration.intrinsics.matrix[0][0] / self.px_per_m,
                    )
            camera_groups[uid].move(*camera.calibration.extrinsics.translation)
            camera_groups[uid].rotate_R(camera.calibration.rotation.R)
        return False  # NOTE: avoid JustPy page_update

    async def update_images(self, _) -> bool:
        newest_images = [camera.latest_captured_image for camera in self.calibrated_cameras.values()]
        newest_images = [i for i in newest_images if i is not None]

        current_uids = [image.camera_id for image in newest_images]
        for uid, texture in list(self.textures.items()):
            if uid not in current_uids:
                texture.delete()
                del self.textures[uid]

        z = 0
        for image in reversed(sorted(newest_images, key=lambda i: i.time)):
            camera = self.calibrated_cameras.get(image.camera_id)
            if camera is None or camera.projection is None:
                continue
            coordinates = [[point and [point[0], point[1], 0] for point in row] for row in camera.projection]

            if image.camera_id not in self.textures:
                self.textures[image.camera_id] = Texture(image.url, coordinates).with_name(f'image_{image.id}')
            texture = self.textures[image.camera_id]

            z += 0.001
            texture.move(z=z)
            if texture.args[0] != image.url:
                await texture.set_url(image.url)
            if not CameraProjector.allclose(texture.args[1], coordinates):
                await texture.set_coordinates(coordinates)
        return False  # NOTE: avoid JustPy page_update
