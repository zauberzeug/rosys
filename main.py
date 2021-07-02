#!/usr/bin/env python3
from controller.esptool import run
from nicegui import ui
import os
from rosys.runtime import Runtime
from rosys.world.mode import Mode
from rosys.ui.joystick import Joystick
from rosys.ui.three import Three
import base64

import icecream
icecream.install()

has_esp = os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)
#runtime = Runtime(Mode.SIMULATION)

state = ui.label()
ui.timer(0.1, lambda: state.set_text(f'''
    {runtime.world.time:.3f} s
    (x={runtime.world.robot.pose.x:.3f},
    y={runtime.world.robot.pose.y:.3f})
'''))

ui.button('Download images', on_click=lambda: runtime.world.download_queue.extend(runtime.world.cameras.keys()))

ui.label('Cameras')

cams = ui.label()
ui.timer(1, lambda: cams.set_text(f'cams: {runtime.world.cameras}'))

with ui.card().style('width:600px'):
    cam_image = ui.image()


def update_camera_images():
    if len(runtime.world.images) > 0:
        image = runtime.world.images[0]
        if hasattr(cam_image, 'id') and cam_image.id == image.id:
            return  # we do not need to update the same image again
    else:
        cam_image.source = None
        cam_image.id = None
        return
    data = runtime.world.image_data.get(image.id, None)
    if data is not None:
        encoded = base64.b64encode(data).decode("utf-8")
        cam_image.source = 'data:image/jpeg;base64,' + encoded
        cam_image.id = image.id
        ic(image.detections)


ui.timer(1, lambda: update_camera_images())


def set_height(height):
    for camera in runtime.world.cameras.values():
        try:
            camera.calibration.extrinsics.translation[2] = height
        except AttributeError:
            pass


ui.number('Height [m]', on_change=lambda e: set_height(e.value))

Joystick(size=50, color='blue', steerer=runtime.steerer)

three = Three(runtime.world.robot.pose)
ui.timer(0.05, lambda: three.set_robot_pose(runtime.world.robot.pose))

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())
