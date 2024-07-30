#!/usr/bin/env python3
import math
from typing import Any

from nicegui import ui

from rosys import config, persistence
from rosys.driving import Driver, Odometer, Steerer, joystick, keyboard_control
from rosys.geometry import Frame3d, Point3d, Pose3d, Rotation
from rosys.hardware import RobotSimulation, WheelsSimulation

wheels = WheelsSimulation()
robot = RobotSimulation([wheels])
steerer = Steerer(wheels)
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)


class Link(persistence.PersistentModule):

    def __init__(self, name: str, parent_frame: Frame3d, *, length: float) -> None:
        super().__init__(persistence_key=name)
        self.base = Frame3d(Pose3d.zero(), parent=parent_frame, id=f'{name}_base')
        self.end = Frame3d(Pose3d(translation=Point3d(x=0, y=0, z=length), rotation=Rotation.zero()),
                           parent=self.base, id=f'{name}_end')
        self.length = length

    def pitch(self, angle: float) -> None:
        self.base.pose.rotation = Rotation.from_euler(0, angle, 0)
        self.request_backup()

    def backup(self) -> dict[str, Any]:
        return {'pose': persistence.to_dict(self.base.pose)}

    def restore(self, data: dict[str, Any]) -> None:
        self.base.pose = persistence.from_dict(Pose3d, data['pose'])


class Cam(persistence.PersistentModule):

    def __init__(self, parent_frame: Frame3d) -> None:
        super().__init__()
        self.pose = Pose3d(translation=Point3d(x=0, y=0, z=0.1), rotation=Rotation.zero(), frame=parent_frame)

    def pitch(self, angle: float) -> None:
        self.pose.rotation = Rotation.from_euler(0, angle, 0)
        self.request_backup()

    def backup(self) -> dict[str, Any]:
        return {'pose': persistence.to_dict(self.pose)}

    def restore(self, data: dict[str, Any]) -> None:
        print(data)
        self.pose = persistence.from_dict(Pose3d, data['pose'])


# Link.USE_PERSISTENCE = False
# Cam.USE_PERSISTENCE = False

robot_frame = Frame3d(pose=Pose3d.zero())
anchor_frame = Frame3d(pose=Pose3d(translation=Point3d(x=0, y=0, z=0.3), rotation=Rotation.zero()), parent=robot_frame)
arm1 = Link('arm1', anchor_frame, length=0.3)
arm2 = Link('arm2', arm1.end, length=0.3)
cam = Cam(arm2.end)


def handle_robot_move():
    robot_frame.pose.translation.x = odometer.prediction.x
    robot_frame.pose.translation.y = odometer.prediction.y
    robot_frame.pose.rotation = Rotation.from_euler(0, 0, odometer.prediction.yaw)


odometer.ROBOT_MOVED.register(handle_robot_move)


@ui.page('/')
def page():
    def update_scene():
        chassis.move(*robot_frame.world_pose.translation.tuple)
        chassis.rotate_R(robot_frame.world_pose.rotation.R)
        segment1.move(*arm1.base.world_pose.translation.tuple)
        segment1.rotate_R(arm1.base.world_pose.rotation.R)
        segment2.move(*arm2.base.world_pose.translation.tuple)
        segment2.rotate_R(arm2.base.world_pose.rotation.R)
        camera_box.move(*cam.pose.resolve().translation.tuple)
        camera_box.rotate_R(cam.pose.resolve().rotation.R)
    ui.timer(config.ui_update_interval, update_scene)

    keyboard_control(steerer)
    with ui.row():
        with ui.scene() as scene:
            with scene.group() as chassis:
                scene.box(width=1.0, height=0.5, depth=0.3).move(z=0.15).material(color='gray')
            with scene.group() as segment1:
                scene.box(width=0.1, height=0.1, depth=arm1.length).move(z=arm1.length / 2)
            with scene.group() as segment2:
                scene.box(width=0.1, height=0.1, depth=arm2.length).move(z=arm2.length / 2)
            with scene.group() as camera_box:
                scene.box(width=0.1, height=0.1, depth=0.1).material(color='SteelBlue')
            scene.move_camera(y=-2, z=2)
        with ui.column():
            joystick(steerer, size=50, color='blue')
            ui.slider(min=-math.pi / 2, max=math.pi / 2, step=0.01, value=0, on_change=lambda e: arm1.pitch(e.value))
            ui.slider(min=-math.pi / 2, max=math.pi / 2, step=0.01, value=0, on_change=lambda e: arm2.pitch(e.value))
            ui.slider(min=-math.pi / 4, max=math.pi / 4, step=0.01, value=0, on_change=lambda e: cam.pitch(e.value))


ui.run(title='Camera Arm')
