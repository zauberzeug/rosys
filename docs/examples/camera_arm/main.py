#!/usr/bin/env python3
import math
from typing import Any

from nicegui import ui

from rosys import config, persistence
from rosys.driving import Driver, Odometer, Steerer, joystick, keyboard_control
from rosys.geometry import Frame3d, Pose3d, Rotation, axes_object
from rosys.hardware import RobotSimulation, WheelsSimulation

wheels = WheelsSimulation()
robot = RobotSimulation([wheels])
steerer = Steerer(wheels)
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)


class Link(persistence.PersistentModule):

    def __init__(self, name: str, parent_frame: Frame3d, *, length: float) -> None:
        super().__init__(persistence_key=name)
        self.name = name
        self.length = length
        self.base = Pose3d().in_frame(parent_frame).as_frame(f'{name}_base')
        self.end = Pose3d(z=length).as_frame(f'{name}_end').in_frame(self.base)

    def pitch(self, angle: float) -> None:
        self.base.rotation = Rotation.from_euler(0, angle, 0)
        self.request_backup()

    def backup(self) -> dict[str, Any]:
        return {
            'name': self.name,
            'base': persistence.to_dict(self.base),
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.name = data['name']
        self.base = persistence.from_dict(Frame3d, data['base'])
        self.end = Pose3d(z=self.length) \
            .as_frame(f'{self.name}_end') \
            .in_frame(self.base)


class Cam(persistence.PersistentModule):

    def __init__(self, parent_frame: Frame3d) -> None:
        super().__init__(persistence_key='cam')
        self.pose = Pose3d(z=0.05).in_frame(parent_frame)

    def pitch(self, angle: float) -> None:
        self.pose.rotation = Rotation.from_euler(0, angle, 0)
        self.request_backup()

    def backup(self) -> dict[str, Any]:
        return {'pose': persistence.to_dict(self.pose)}

    def restore(self, data: dict[str, Any]) -> None:
        self.pose = persistence.from_dict(Frame3d, data['pose'])


anchor_frame = Pose3d(z=0.3).as_frame('anchor').in_frame(odometer.prediction_frame)
arm1 = Link('arm1', anchor_frame, length=0.3)
arm2 = Link('arm2', arm1.end, length=0.3)
cam = Cam(arm2.end)


@ui.page('/')
def page():
    def update_scene():
        chassis.move(*odometer.prediction_frame.resolve().translation)
        chassis.rotate_R(odometer.prediction_frame.resolve().rotation.R)
        segment1.move(*arm1.base.resolve().translation)
        segment1.rotate_R(arm1.base.resolve().rotation.R)
        segment2.move(*arm2.base.resolve().translation)
        segment2.rotate_R(arm2.base.resolve().rotation.R)
        camera_box.move(*cam.pose.resolve().translation)
        camera_box.rotate_R(cam.pose.resolve().rotation.R)
    ui.timer(config.ui_update_interval, update_scene)

    keyboard_control(steerer)
    with ui.row():
        with ui.scene() as scene:
            with scene.group() as chassis:
                scene.box(width=1.0, height=0.5, depth=0.3) \
                    .move(z=0.15).material(color='gray')
            with scene.group() as segment1:
                scene.box(width=0.1, height=0.1, depth=arm1.length) \
                    .move(z=arm1.length / 2)
            with scene.group() as segment2:
                scene.box(width=0.1, height=0.1, depth=arm2.length) \
                    .move(z=arm2.length / 2)
            with scene.group() as camera_box:
                scene.box(width=0.1, height=0.1, depth=0.1) \
                    .material(color='SteelBlue')
            scene.move_camera(y=-1, z=1, look_at_z=0.5)
        with ui.column():
            joystick(steerer, size=50, color='blue')
            ui.slider(min=-math.pi / 2, max=math.pi / 2, step=0.01, value=0,
                      on_change=lambda e: arm1.pitch(e.value))
            ui.slider(min=-math.pi / 2, max=math.pi / 2, step=0.01, value=0,
                      on_change=lambda e: arm2.pitch(e.value))
            ui.slider(min=-math.pi / 4, max=math.pi / 4, step=0.01, value=0,
                      on_change=lambda e: cam.pitch(e.value))
        with ui.scene() as scene2:
            axes_object(anchor_frame, length=0.15)
            axes_object(arm1.base, name='Arm 1 Base', length=0.15)
            axes_object(arm2.base, name='Arm 2 Base', length=0.15)
            axes_object(cam.pose, name='Camera', length=0.15)
            scene2.move_camera(y=-1, z=1, look_at_z=0.5)


ui.run(title='Camera Arm')
