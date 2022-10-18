from __future__ import annotations

from nicegui import ui
from nicegui.elements.scene_objects import Group, Sphere
from rosys import config

from .driver import Driver


class DriverObject(Group):
    '''The DriverObject UI element displays the path following process in a 3D scene.

    The current pose is taken from a given odometer.
    An optional driver module shows debugging information about a current path-following process.
    The `debug` argument can be set to show a wireframe instead of a closed polygon.
    '''

    def __init__(self, driver: Driver) -> None:
        super().__init__()
        self.driver = driver
        with self:
            with Group() as self.robot_group:
                Sphere(0.03).material('#4488ff')
                Sphere(0.05).material('#4488ff').move(self.driver.parameters.hook_offset)
            with Group() as self.carrot_group:
                Sphere(0.03).material('#ff8800')
                Sphere(0.05).material('#ff8800').move(self.driver.parameters.carrot_offset)
        ui.timer(config.ui_update_interval, self.update)

    def update(self) -> None:
        robot_pose = self.driver.odometer.prediction
        self.robot_group.move(robot_pose.x, robot_pose.y).rotate(0, 0, robot_pose.yaw)

        carrot_pose = self.driver.carrot_pose
        if carrot_pose:
            self.carrot_group.scale(1).move(carrot_pose.x, carrot_pose.y).rotate(0, 0, carrot_pose.yaw)
        else:
            self.carrot_group.scale(0)
