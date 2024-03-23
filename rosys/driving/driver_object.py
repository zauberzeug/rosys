from __future__ import annotations

import numpy as np
from nicegui import ui
from nicegui.elements.scene_objects import Group, Sphere

from ..rosys import config
from .driver import Driver


class DriverObject(Group):
    """The DriverObject UI element displays the path following process in a 3D scene.

    The current pose is taken from a given odometer.
    An optional driver module shows debugging information about a current path-following process.
    The `debug` argument can be set to show a wireframe instead of a closed polygon.
    """

    def __init__(self, driver: Driver) -> None:
        super().__init__()
        self.driver = driver
        with self:
            with Group() as self.robot_group:
                Sphere(0.03).material('#4488ff')
                self.hook = Sphere(0.05).material('#4488ff')
                self.pull = Sphere(0.05).material('#ffbb00')
            with Group() as self.carrot_group:
                Sphere(0.03).material('#ff8800')
                Sphere(0.05).material('#ff8800').move(self.driver.parameters.carrot_offset)
        ui.timer(config.ui_update_interval, self.update)

    def update(self) -> None:
        robot_pose = self.driver.prediction
        self.robot_group.move(robot_pose.x, robot_pose.y).rotate(0, 0, robot_pose.yaw)

        if self.driver.state:
            self.scale(1)
            carrot_pose = self.driver.state.carrot_pose
            self.carrot_group.move(carrot_pose.x, carrot_pose.y).rotate(0, 0, carrot_pose.yaw)
            angle = self.driver.state.turn_angle + (np.pi if self.driver.state.backward else 0)
            hook_offset = self.driver.parameters.hook_offset * (-1 if self.driver.state.backward else 1)
            pull_distance = 1.5 * self.driver.parameters.carrot_distance
            self.hook.move().move(hook_offset)
            self.pull.move(hook_offset + np.cos(angle) * pull_distance, np.sin(angle) * pull_distance)
        else:
            self.scale(0)
