#!/usr/bin/env python3
import os
import uuid

import rosys
from nicegui import ui
from rosys.automation import AutomationControls, Automator
from rosys.driving import Driver, KeyboardControl, Odometer, PathSegment, RobotObject, RobotShape, Steerer
from rosys.geometry import Point, Pose, Spline
from rosys.hardware import RobotBrain, SerialCommunication, WheelsHardware, WheelsSimulation
from rosys.pathplanning import Obstacle, ObstacleObject, PathPlanner
from rosys.pathplanning.path_object import PathObject

# setup
shape = RobotShape(outline=[(0, 0), (-0.5, -0.5), (1.5, -0.5), (1.75, 0), (1.5, 0.5), (-0.5, 0.5)])
odometer = Odometer()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(odometer, robot_brain)
else:
    wheels = WheelsSimulation(odometer)
steerer = Steerer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(wheels, steerer)
path_planner = PathPlanner(shape)


# ui
rosys.NEW_NOTIFICATION.register(ui.notify)
KeyboardControl(steerer)

with ui.card():
    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'{rosys.time():.3f} s, {odometer.prediction}'))

    click_mode = ui.toggle({
        'drive': 'Drive',
        'navigate': 'Navigate',
        'obstacles': 'Obstacles',
    }, value='drive').props('outline')

    async def handle_click(msg):
        if msg.click_type != 'dblclick':
            return
        for hit in msg.hits:
            object_type = hit.object_id if hit.object is None else (hit.object.name or '').split('_')[0]
            if object_type == 'ground' and click_mode.value == 'drive':
                start = odometer.prediction.point
                target = Point(x=hit.point.x, y=hit.point.y)
                path = [PathSegment(spline=Spline(
                    start=start,
                    control1=start.interpolate(target, 1/3),
                    control2=start.interpolate(target, 2/3),
                    end=hit.point,
                ))]
                path3d.update(path)
                automator.start(driver.drive_path(path))
                return
            if object_type == 'ground' and click_mode.value == 'navigate':
                goal = Pose(x=hit.point.x, y=hit.point.y, yaw=odometer.prediction.point.direction(hit.point))
                path = await path_planner.search(start=odometer.prediction, goal=goal, timeout=3.0)
                path3d.update(path)
                automator.start(driver.drive_path(path))
                return
            if object_type == 'ground' and click_mode.value == 'obstacles':
                id = str(uuid.uuid4())
                path_planner.obstacles[id] = Obstacle(id=id, outline=[
                    Point(x=hit.point.x-0.5, y=hit.point.y-0.5),
                    Point(x=hit.point.x+0.5, y=hit.point.y-0.5),
                    Point(x=hit.point.x+0.5, y=hit.point.y+0.5),
                    Point(x=hit.point.x-0.5, y=hit.point.y+0.5),
                ])
                path_planner.needs_backup = True
                obstacles3d.update()
                return
            if object_type == 'obstacle' and click_mode.value == 'obstacles':
                del path_planner.obstacles[hit.object.name.split('_')[1]]
                path_planner.needs_backup = True
                obstacles3d.update()
                return

    with ui.scene(640, 480, on_click=handle_click) as scene:
        RobotObject(shape, odometer, debug=True)
        obstacles3d = ObstacleObject(path_planner.obstacles)
        path3d = PathObject()

    with ui.row():
        AutomationControls(automator)
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

# start
ui.on_startup(rosys.startup)
ui.on_shutdown(rosys.shutdown)
ui.run(title='obstacles', port=8080)
