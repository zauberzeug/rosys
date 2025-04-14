#!/usr/bin/env python3
from nicegui import ui
from nicegui.events import SceneClickEventArguments

from rosys.automation import Automator
from rosys.driving import Driver, Odometer, robot_object
from rosys.geometry import Point, Pose, Prism
from rosys.hardware import RobotSimulation, WheelsSimulation
from rosys.pathplanning import Obstacle, PathPlanner, obstacle_object, path_object

shape = Prism.default_robot_shape()
PathPlanner.USE_PERSISTENCE = False
path_planner = PathPlanner(shape)
path_planner.obstacles['0'] = Obstacle(id='0', outline=[Point(x=3, y=0),
                                                        Point(x=0, y=3),
                                                        Point(x=3, y=3)])
wheels = WheelsSimulation()
robot = RobotSimulation([wheels])
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(None, on_interrupt=wheels.stop)


async def handle_click(e: SceneClickEventArguments):
    for hit in e.hits:
        if hit.object_id == 'ground':
            goal = Pose(x=hit.x, y=hit.y,
                        yaw=odometer.prediction.direction(Point(x=hit.x, y=hit.y)))
            path = await path_planner.search(start=odometer.prediction, goal=goal)
            path3d.update(path)
            automator.start(driver.drive_path(path))

with ui.scene(on_click=handle_click, width=600):
    robot_object(shape, odometer)
    obstacle_object(path_planner)
    path3d = path_object()

ui.label('click into the scene to drive the robot')

ui.run(title='RoSys')
