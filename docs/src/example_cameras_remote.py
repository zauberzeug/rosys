#!/usr/bin/env python3
from nicegui import ui
from rosys.driving import Joystick, KeyboardControl, Odometer, Steerer
from rosys.hardware import RobotBrain, SerialCommunication, WheelsHardware, WheelsSimulation
from rosys.vision import Camera, CameraProvider, CameraServer, UsbCameraProviderHardware, UsbCameraProviderSimulation

if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(robot_brain)
    camera_provider = UsbCameraProviderHardware()
else:
    wheels = WheelsSimulation()
    camera_provider = UsbCameraProviderSimulation()
    camera_provider.restore = lambda _: None  # NOTE: disable persistence
    test_cam = camera_provider.create_calibrated('test_cam', width=800, height=600)
    ui.on_startup(lambda: camera_provider.add_camera(test_cam))
CameraServer(camera_provider)
steerer = Steerer(wheels)
odometer = Odometer(wheels)


async def add_main_camera(camera: Camera) -> None:
    camera_card.clear()  # remove "seeking camera" label
    with camera_card:
        maincam = ui.image()
        ui.timer(1, lambda: maincam.set_source(camera.latest_image_uri))

camera_provider.CAMERA_ADDED.register(add_main_camera)

with ui.card().tight().style('width:30em') as camera_card:
    ui.label('seeking main camera').classes('m-8 text-center')

with ui.card().tight().style('width:30em'):
    with ui.row():
        with ui.card().tight():
            Joystick(steerer)
            KeyboardControl(steerer)
        ui.markdown('steer with joystick on the left or<br />SHIFT + arrow keys').classes('m-8 text-center')

ui.run(title='RoSys')
