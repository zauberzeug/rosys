#!/usr/bin/env python3
import math

from nicegui import ui
from nicegui.elements.scene_objects import Box

import rosys
from rosys.geometry import CoordinateFrame, Point3d, Pose3d, Rotation, scene_object

frame = CoordinateFrame.from_pose(Pose3d.zero())
pose = Pose3d.zero()
pose.parent_frame = frame


def step():
    frame.rotation = Rotation.from_euler(0, 0, rosys.time() / 2.0)
    pose.translation = Point3d(x=math.sin(rosys.time()), y=0, z=0.75)


rosys.on_repeat(step, interval=0.01)

with ui.scene():
    scene_object(lambda: Box(width=1, height=1, depth=1).material(color='#0000ff'), pose=frame)
    scene_object(lambda: Box(width=0.5, height=0.5, depth=0.5).material(color='#ff0000'), pose=pose)


def set_parent():
    pose.parent_frame = frame


def reset_parent():
    pose.parent_frame = None


with ui.row():
    ui.button('Parent Frame', on_click=set_parent)
    ui.button('Parent World', on_click=reset_parent)

ui.run(title='RoSys')
