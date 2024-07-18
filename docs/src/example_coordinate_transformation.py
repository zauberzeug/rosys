#!/usr/bin/env python3
from nicegui import ui
from nicegui.elements.scene_objects import Extrusion

from rosys.driving import Odometer, Steerer, joystick, keyboard_control
from rosys.geometry import Point3d, Pose3d, Prism, Rotation, scene_object
from rosys.hardware import RobotSimulation, WheelsSimulation
from rosys.vision import CalibratableCamera, CameraSceneObject, SimulatedCamera

wheels = WheelsSimulation()
robot = RobotSimulation([wheels])
odometer = Odometer(wheels)
steerer = Steerer(wheels)

keyboard_control(steerer)
joystick(steerer, size=50, color='blue')


robot_shape = Prism.default_robot_shape()
robot_pose = Pose3d.zero()

hat_shape = Prism.default_robot_shape()
hat_pose = Pose3d(translation=Point3d(x=-0.5, y=0, z=1),
                  rotation=Rotation.from_euler(0, -1.5, 0), parent_frame_pose=robot_pose)


class CalibratableCameraSimulated(SimulatedCamera, CalibratableCamera):
    pass


camera = CalibratableCameraSimulated.create_calibrated(id='test', name='Test Camera', x=0.75, y=0, z=0.5)
assert camera.calibration is not None
camera.calibration.extrinsics.parent_frame_pose = hat_pose


def update_pose():
    robot_pose.translation = Point3d(x=odometer.prediction.x, y=odometer.prediction.y, z=0)
    robot_pose.rotation = Rotation.from_euler(0, 0, odometer.prediction.yaw)
    return robot_pose


def shape_object(shape: Prism, debug: bool = False) -> Extrusion:
    outline = [list(point) for point in shape.outline]
    return Extrusion(outline, shape.height, wireframe=debug)


with ui.scene():
    scene_object(lambda: shape_object(robot_shape), robot_pose)
    scene_object(lambda: shape_object(hat_shape, debug=True), hat_pose)
    scene_object(lambda camera=camera: CameraSceneObject(camera), camera.calibration.extrinsics)

odometer.ROBOT_MOVED.register(update_pose)


def update_rotation(x):
    camera.calibration.extrinsics.rotation = Rotation.from_euler(x, x, x)


ui.slider(min=-1.5, max=1.5, step=0.01).on_value_change(lambda event: update_rotation(event.value))
ui.slider(min=-1, max=1, step=0.01).bind_value(camera.calibration.extrinsics.translation, 'x')

ui.run(title='RoSys')
