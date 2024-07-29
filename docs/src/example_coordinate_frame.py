#!/usr/bin/env python3
import math

from nicegui import ui

import rosys
from rosys.geometry import Frame3d, Point3d, Rotation
from rosys.vision import CalibratableCamera, CameraSceneObject, SimulatedCamera


class SimulatedCalibratableCamera(SimulatedCamera, CalibratableCamera):
    pass


blue_frame = Frame3d(translation=Point3d(x=0, y=0, z=0.5), rotation=Rotation.zero())
pink_frame = Frame3d(translation=Point3d(x=0, y=0, z=0.75), rotation=Rotation.zero(), parent_frame=blue_frame)
camera = SimulatedCalibratableCamera.create_calibrated(id='Camera', z=0.5, roll=math.pi / 2, parent_frame=pink_frame)

with ui.scene() as scene:
    blue_box = scene.box(width=1, height=1, depth=1).material(color='SteelBlue')
    pink_box = scene.box(width=0.5, height=0.5, depth=0.5).material(color='HotPink')
    camera_object = CameraSceneObject(camera)


def update():
    blue_frame.translation.x = math.cos(0.5 * rosys.time())
    blue_frame.translation.y = math.sin(0.5 * rosys.time())
    pink_frame.rotate(Rotation.from_euler(0, 0, 0.005))

    blue_pose = blue_frame.resolve()
    blue_box.rotate_R(blue_pose.rotation.R)
    blue_box.move(*blue_pose.translation.tuple)

    pink_pose = pink_frame.resolve()
    pink_box.rotate_R(pink_pose.rotation.R)
    pink_box.move(*pink_pose.translation.tuple)

    camera_pose = camera.calibration.extrinsics.resolve()
    camera_object.rotate_R(camera_pose.rotation.R)
    camera_object.move(*camera_pose.translation.tuple)


rosys.on_repeat(update, interval=0.01)


def set_parent(frame: Frame3d | None) -> None:
    pink_frame.parent_frame = frame


ui.button('Toggle frame', on_click=lambda: set_parent(None if pink_frame.parent_frame == blue_frame else blue_frame))

ui.run(title='RoSys')
