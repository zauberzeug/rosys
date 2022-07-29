#!/usr/bin/env python3
from nicegui import ui
from rosys.automation import Automator
from rosys.driving import Driver, Odometer, RobotObject, RobotShape
from rosys.geometry import Pose
from rosys.hardware import WheelsSimulation
from rosys.pathplanning import PathObject, PathPlanner

# setup
shape = RobotShape()
odometer = Odometer()
path_planner = PathPlanner(shape)
wheels = WheelsSimulation(odometer)
driver = Driver(wheels, odometer)
automator = Automator(wheels, None)


async def handle_click(msg):
    for hit in msg.hits:
        yaw = odometer.prediction.point.direction(hit.point)
        path = await path_planner.search(start=odometer.prediction, goal=Pose(x=hit.point.x, y=hit.point.y, yaw=yaw))
        path3d.update(path)
        automator.start(driver.drive_path(path))

# ui
with ui.scene(on_click=handle_click, width=600):
    RobotObject(shape, odometer)
    path3d = PathObject()

ui.label('click into the scene to drive the robot')

# start
ui.run(title='RoSys')
