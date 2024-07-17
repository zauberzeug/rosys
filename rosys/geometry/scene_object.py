from __future__ import annotations

from nicegui import ui
from nicegui.elements.scene_objects import Extrusion, Group, Stl

from ..geometry import Prism
from ..rosys import config
from .pose3d import Pose3d


class scene_object(Group):
    """The RobotObject UI element displays the robot with its given shape in a 3D scene.

    The current pose is taken from a given odometer.
    The `debug` argument can be set to show a wireframe instead of a closed polygon.
    """

    def __init__(self, shape: Prism, pose: Pose3d, *, debug: bool = False) -> None:
        super().__init__()
        self.shape = shape
        self.pose = pose
        self.robot_object: Extrusion | Stl
        with self:
            outline = [list(point) for point in self.shape.outline]
            self.robot_object = Extrusion(outline, self.shape.height, wireframe=debug)
            self.robot_object.material('#4488ff', 0.5)
        ui.timer(config.ui_update_interval, self.update)

    def with_stl(self, url: str, *,
                 x: float = 0, y: float = 0, z: float = 0,
                 omega: float = 0, phi: float = 0, kappa: float = 0,
                 scale: float = 1.0,
                 color: str = '#ffffff', opacity: float = 1.0) -> scene_object:
        """Sets an STL to be displayed as the robot.

        The file can be served from a local directory with [app.add_static_files(url, path)](https://nicegui.io/reference#static_files).
        """
        self.robot_object.delete()
        with self:
            self.robot_object = Stl(url).move(x, y, z).rotate(omega, phi, kappa).scale(scale).material(color, opacity)
        return self

    def update(self) -> None:
        pose_in_world = self.pose.resolve()
        roll, pitch, yaw = pose_in_world.rotation.euler
        self.rotate(roll, pitch, yaw)
        self.move(pose_in_world.translation.x, pose_in_world.translation.y, pose_in_world.translation.z)
