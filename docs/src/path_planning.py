#!/usr/bin/env python3
import rosys.ui
from nicegui import ui
from rosys import runtime
from rosys.actors import Automator, Driver, Odometer, PathPlanner
from rosys.hardware import WheelsSimulation
from rosys.world import Pose, RobotShape

# setup
shape = RobotShape()
odometer = Odometer()
path_planner = PathPlanner(shape)
wheels = WheelsSimulation(odometer)
driver = Driver(wheels, odometer)
automator = Automator()


async def handle_click(msg):
    for hit in msg.hits:
        yaw = odometer.prediction.point.direction(hit.point)
        path = await path_planner.search(start=odometer.prediction, goal=Pose(x=hit.point.x, y=hit.point.y, yaw=yaw))
        path3d.update(path)
        automator.start(driver.drive_path(path))

# ui
runtime.NEW_NOTIFICATION.register(ui.notify)
with ui.scene(on_click=handle_click, width=600):
    rosys.ui.robot_object(shape, odometer)
    path3d = rosys.ui.path_object()

ui.label('click into the scene to drive the robot')

# start
ui.on_startup(runtime.startup)
ui.on_shutdown(runtime.shutdown)
ui.run(title='RoSys')
