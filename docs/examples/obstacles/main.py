#!/usr/bin/env python3
import os
import uuid

from nicegui import ui
from nicegui.events import SceneClickEventArguments

import rosys
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Odometer, PathSegment, Steerer, keyboard_control, robot_object
from rosys.geometry import Point, Pose, Prism, Spline
from rosys.hardware import (
    CanHardware,
    RobotBrain,
    RobotHardware,
    RobotSimulation,
    SerialCommunication,
    WheelsHardware,
    WheelsSimulation,
)
from rosys.pathplanning import Obstacle, PathPlanner, obstacle_object, path_object

# setup
shape = Prism(outline=[(0, 0), (-0.5, -0.5), (1.5, -0.5), (1.75, 0), (1.5, 0.5), (-0.5, 0.5)], height=0.5)
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    can = CanHardware(robot_brain)
    wheels = WheelsHardware(robot_brain, can=can)
    robot = RobotHardware([can, wheels], robot_brain)
else:
    wheels = WheelsSimulation()
    robot = RobotSimulation([wheels])
steerer = Steerer(wheels)
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(steerer, on_interrupt=wheels.stop)
path_planner = PathPlanner(shape)

# ui
with ui.card():
    keyboard_control(steerer)

    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(
        f'{rosys.time():.3f} s, {odometer.prediction}'
    ))

    click_mode = ui.toggle({
        'drive': 'Drive',
        'navigate': 'Navigate',
        'obstacles': 'Obstacles',
    }, value='drive').props('outline')

    async def handle_click(e: SceneClickEventArguments):
        if e.click_type != 'dblclick':
            return
        for hit in e.hits:
            object_type = (hit.object_name or '').split('_')[0] or hit.object_id
            target = Point(x=hit.x, y=hit.y)
            if object_type == 'ground' and click_mode.value == 'drive':
                start = odometer.prediction.point
                path = [PathSegment(spline=Spline(
                    start=start,
                    control1=start.interpolate(target, 1/3),
                    control2=start.interpolate(target, 2/3),
                    end=target,
                ))]
                path3d.update(path)
                automator.start(driver.drive_path(path))
                return
            if object_type == 'ground' and click_mode.value == 'navigate':
                goal = Pose(x=hit.x, y=hit.y,
                            yaw=odometer.prediction.direction(target))
                path = await path_planner.search(start=odometer.prediction,
                                                 goal=goal, timeout=3.0)
                path3d.update(path)
                automator.start(driver.drive_path(path))
                return
            if object_type == 'ground' and click_mode.value == 'obstacles':
                id_ = str(uuid.uuid4())
                path_planner.obstacles[id_] = Obstacle(id=id_, outline=[
                    Point(x=hit.x-0.5, y=hit.y-0.5),
                    Point(x=hit.x+0.5, y=hit.y-0.5),
                    Point(x=hit.x-0.5, y=hit.y+0.5),
                ])
                path_planner.request_backup()
                path_planner.OBSTACLES_CHANGED.emit(path_planner.obstacles)
                return
            if object_type == 'obstacle' and click_mode.value == 'obstacles':
                del path_planner.obstacles[hit.object.name.split('_')[1]]
                path_planner.request_backup()
                path_planner.OBSTACLES_CHANGED.emit(path_planner.obstacles)
                return

    with ui.scene(640, 480, on_click=handle_click) as scene:
        robot_object(shape, odometer, debug=True)
        obstacle_object(path_planner)
        path3d = path_object()

    with ui.row():
        automation_controls(automator)
        ui.button('restart rosys',
                  on_click=lambda: os.utime('main.py')).props('outline')

# start
ui.run(title='obstacles')
