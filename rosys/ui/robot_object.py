from nicegui.elements.scene_objects import Extrusion, Group, Sphere, Stl
from nicegui.elements.scene_object3d import Object3D
from nicegui.ui import Ui
from ..world import Robot
from .settings import update_interval


class RobotObject(Object3D):
    robot: Robot = None  # will be set by rosys.ui.configure
    ui: Ui = None  # will be set by rosys.ui.configure

    def __init__(self, *, debug: bool = False):
        super().__init__('group')
        with self:
            with Group() as self.robot_group:
                outline = list(map(list, self.robot.shape.outline))
                self.robot_object = Extrusion(outline, self.robot.shape.height, wireframe=debug)
                self.robot_object.material('#4488ff', 0.5)
                if debug:
                    Sphere(0.03).material('#4488ff')
                    Sphere(0.05).material('#4488ff').move(self.robot.parameters.hook_offset)
            with Group() as self.carrot_group:
                Sphere(0.03).material('#ff8800')
                Sphere(0.05).material('#ff8800').move(self.robot.parameters.carrot_offset)
        self.ui.timer(update_interval, self.update)

    def with_stl(self, url: str, *,
                 x: float = 0, y: float = 0, z: float = 0,
                 omega: float = 0, phi: float = 0, kappa: float = 0,
                 scale: float = 1.0,
                 color: str = '#ffffff', opacity: float = 1.0):
        self.robot_object.delete()
        with self.robot_group:
            self.robot_object = Stl(url).move(x, y, z).rotate(omega, phi, kappa).scale(scale).material(color, opacity)
        return self

    def update(self) -> bool:
        self.robot_group.move(self.robot.prediction.x, self.robot.prediction.y)
        self.robot_group.rotate(0, 0, self.robot.prediction.yaw)
        if self.robot.carrot is None:
            self.carrot_group.scale(0)
        else:
            self.carrot_group.scale(1)
            self.carrot_group.move(self.robot.carrot.x, self.robot.carrot.y)
            self.carrot_group.rotate(0, 0, self.robot.carrot.yaw)
        return False  # NOTE: avoid JustPy page_update
