#!/usr/bin/env python3
import math

from nicegui import ui

import rosys
from rosys.geometry import Pose3d, Rotation
from rosys.vision import CalibratableCamera, CameraSceneObject, SimulatedCamera


class SimulatedCalibratableCamera(SimulatedCamera, CalibratableCamera):
    pass


blue = Pose3d(z=0.5).as_frame('blue')
pink = Pose3d(z=0.75).as_frame('pink').in_frame(blue)
camera = SimulatedCalibratableCamera.create_calibrated(id='Camera', z=0.5, roll=math.pi / 2, frame=pink)

with ui.scene() as scene:
    blue_box = scene.box(width=1, height=1, depth=1).material(color='SteelBlue')
    pink_box = scene.box(width=0.5, height=0.5, depth=0.5).material(color='HotPink')
    camera_object = CameraSceneObject(camera)


def update():
    blue.x = math.cos(0.5 * rosys.time())
    blue.y = math.sin(0.5 * rosys.time())
    pink.rotation *= Rotation.from_euler(0, 0, 0.005)

    blue_box.rotate_R(blue.resolve().rotation.R)
    blue_box.move(*blue.resolve().translation)

    pink_box.rotate_R(pink.resolve().rotation.R)
    pink_box.move(*pink.resolve().translation)

    camera_object.rotate_R(camera.calibration.extrinsics.resolve().rotation.R)
    camera_object.move(*camera.calibration.extrinsics.resolve().translation)


rosys.on_repeat(update, interval=0.01)

ui.button('Toggle frame', on_click=lambda: pink.in_frame(None if pink.frame_id == 'blue' else blue))

ui.run(title='RoSys')
