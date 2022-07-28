#!/usr/bin/env python3
import rosys.ui
from nicegui import ui
import rosys
from rosys.actors import (CameraProvider, CameraServer, Odometer, Steerer, UsbCameraProviderHardware,
                          UsbCameraProviderSimulation)
from rosys.hardware import RobotBrain, WheelsHardware, WheelsSimulation
from rosys.hardware.communication import SerialCommunication
from rosys.world import Camera

# setup
odometer = Odometer()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(odometer, robot_brain)
    camera_provider = UsbCameraProviderHardware()
else:
    wheels = WheelsSimulation(odometer)
    camera_provider = UsbCameraProviderSimulation()
CameraServer(camera_provider)
steerer = Steerer(wheels)


async def add_main_camera(camera: Camera) -> None:
    camera_card.clear()  # remove "seeking camera" label
    with camera_card:
        maincam = ui.image()
        ui.timer(1, lambda: maincam.set_source(camera.latest_image_uri))

CameraProvider.CAMERA_ADDED.register(add_main_camera)
ui.on_startup(lambda: camera_provider.add_camera(camera_provider.create_calibrated('test_cam', width=800, height=600)))

# ui
with ui.card().tight().style('width:30em') as camera_card:
    ui.label('seeking main camera').classes('m-8 text-center')

with ui.card().tight().style('width:30em'):
    with ui.row():
        with ui.card().tight():
            rosys.ui.joystick(steerer)
            rosys.ui.keyboard_control(steerer)
        ui.markdown('steer with joystick on the left or<br />SHIFT + arrow keys').classes('m-8 text-center')

# start
ui.on_startup(rosys.startup)
ui.on_shutdown(rosys.shutdown)
ui.run(title='RoSys')
