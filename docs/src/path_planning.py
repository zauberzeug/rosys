#!/usr/bin/env python3
from nicegui import ui
from rosys.automation import Automator
from rosys.driving import Driver, Odometer, RobotObject
from rosys.geometry import Pose, Prism
from rosys.hardware import WheelsSimulation
from rosys.pathplanning import PathObject, PathPlanner

# setup
shape = Prism.default_robot_shape()
path_planner = PathPlanner(shape)
wheels = WheelsSimulation()
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(wheels, None)


async def handle_click(msg):
    for hit in msg.hits:
        if hit.object_id == 'ground':
            yaw = odometer.prediction.point.direction(hit.point)
            goal = Pose(x=hit.point.x, y=hit.point.y, yaw=yaw)
            path = await path_planner.search(start=odometer.prediction, goal=goal)
            path3d.update(path)
            automator.start(driver.drive_path(path))

# ui
with ui.scene(on_click=handle_click, width=600):
    RobotObject(shape, odometer)
    path3d = PathObject()

ui.label('click into the scene to drive the robot')

# start
ui.run(title='RoSys')
