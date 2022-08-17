from __future__ import annotations

from typing import Optional

from nicegui import ui
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Extrusion, Group, Sphere, Stl
from rosys import config

from ..geometry import Prism
from .driver import Driver
from .odometer import Odometer


class RobotObject(Object3D):
    '''The RobotObject UI element displays the robot with its given shape in a 3D scene.

    The current pose is taken from a given odometer.
    An optional driver module shows debugging information about a current path-following process.
    The `debug` argument can be set to show a wireframe instead of a closed polygon.
    '''

    def __init__(self, shape: Prism, odometer: Odometer, driver: Optional[Driver] = None, *,
                 debug: bool = False) -> None:
        super().__init__('group')
        self.shape = shape
        self.odometer = odometer
        self.driver = driver
        with self:
            with Group() as self.robot_group:
                outline = list(map(list, self.shape.outline))
                self.robot_object = Extrusion(outline, self.shape.height, wireframe=debug)
                self.robot_object.material('#4488ff', 0.5)
                if debug:
                    Sphere(0.03).material('#4488ff')
                    if self.driver:
                        Sphere(0.05).material('#4488ff').move(self.driver.parameters.hook_offset)
            with Group() as self.carrot_group:
                Sphere(0.03).material('#ff8800')
                if self.driver:
                    Sphere(0.05).material('#ff8800').move(self.driver.parameters.carrot_offset)
        ui.timer(config.ui_update_interval, self.update)

    def with_stl(self, url: str, *,
                 x: float = 0, y: float = 0, z: float = 0,
                 omega: float = 0, phi: float = 0, kappa: float = 0,
                 scale: float = 1.0,
                 color: str = '#ffffff', opacity: float = 1.0) -> RobotObject:
        self.robot_object.delete()
        with self.robot_group:
            self.robot_object = Stl(url).move(x, y, z).rotate(omega, phi, kappa).scale(scale).material(color, opacity)
        return self

    def update(self) -> None:
        self.robot_group.move(self.odometer.prediction.x, self.odometer.prediction.y)
        self.robot_group.rotate(0, 0, self.odometer.prediction.yaw)
        if self.driver is None or self.driver.carrot_pose is None:
            self.carrot_group.scale(0)
        else:
            self.carrot_group.scale(1)
            self.carrot_group.move(self.driver.carrot_pose.x, self.driver.carrot_pose.y)
            self.carrot_group.rotate(0, 0, self.driver.carrot_pose.yaw)
