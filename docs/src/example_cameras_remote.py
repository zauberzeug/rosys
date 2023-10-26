#!/usr/bin/env python3
from nicegui import ui

import rosys

if rosys.hardware.SerialCommunication.is_possible():
    communication = rosys.hardware.SerialCommunication()
    robot_brain = rosys.hardware.RobotBrain(communication)
    can = rosys.hardware.CanHardware(robot_brain)
    wheels = rosys.hardware.WheelsHardware(robot_brain, can=can)
    robot = rosys.hardware.RobotHardware([can, wheels], robot_brain)
    camera_provider = rosys.vision.UsbCameraProvider()
else:
    wheels = rosys.hardware.WheelsSimulation()
    robot = rosys.hardware.RobotSimulation([wheels])
    rosys.vision.SimulatedCameraProvider.USE_PERSISTENCE = False
    camera_provider = rosys.vision.SimulatedCameraProvider()
    camera = rosys.vision.SimulatedCamera(id='test_cam', resolution=rosys.vision.ImageSize(width=800, height=600))
    rosys.on_startup(lambda: camera_provider.add_camera(camera))
steerer = rosys.driving.Steerer(wheels)
odometer = rosys.driving.Odometer(wheels)


async def add_main_camera(camera: rosys.vision.Camera) -> None:
    camera_card.clear()  # remove "seeking camera" label
    with camera_card:
        main_cam = ui.interactive_image()
        ui.timer(0.1, lambda: main_cam.set_source(camera.get_latest_image_url()))

camera_provider.CAMERA_ADDED.register_ui(add_main_camera)

with ui.card().tight().style('width:30em') as camera_card:
    ui.label('seeking main camera').classes('m-8 text-center')

with ui.card().tight().style('width:30em'):
    with ui.row():
        with ui.card().tight():
            rosys.driving.joystick(steerer)
            rosys.driving.keyboard_control(steerer)
        ui.markdown('steer with joystick on the left or<br />SHIFT + arrow keys').classes('m-8 text-center')

ui.run(title='RoSys')
