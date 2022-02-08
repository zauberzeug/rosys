#!/usr/bin/env python3
from nicegui import ui
import os
import uuid
import rosys
import rosys.ui
from rosys.automations import drive_path
from rosys.world import Obstacle, PathSegment, Point, Pose, Robot, RobotShape, Spline, World

shape = RobotShape(outline=[(0, 0), (-0.5, -0.5), (1.5, -0.5), (1.75, 0), (1.5, 0.5), (-0.5, 0.5)])
world = World(robot=Robot(shape=shape))
planner = rosys.pathplanning.Planner(world)
runtime = rosys.Runtime(world, rosys.Persistence(world, '~/.rosys/obstacles/world.json'))
rosys.ui.configure(ui, runtime)
rosys.ui.keyboard_control()

with ui.card():
    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'{world.time:.3f} s, {world.robot.prediction}'))

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
                start = world.robot.prediction.point
                target = Point(x=hit.point.x, y=hit.point.y)
                path = [PathSegment(spline=Spline(
                    start=start,
                    control1=start.interpolate(target, 1/3),
                    control2=start.interpolate(target, 2/3),
                    end=hit.point,
                ))]
                path3d.update(path)
                await runtime.automator.start(drive_path(world, runtime.hardware, path))
                return
            if object_type == 'ground' and click_mode.value == 'navigate':
                target_yaw = world.robot.prediction.point.direction(hit.point)
                path = await planner.search_async(goal=Pose(x=hit.point.x, y=hit.point.y, yaw=target_yaw), timeout=3.0)
                path3d.update(path)
                await runtime.automator.start(drive_path(world, runtime.hardware, path))
                return
            if object_type == 'ground' and click_mode.value == 'obstacles':
                id = str(uuid.uuid4())
                world.obstacles[id] = Obstacle(id=id, outline=[
                    Point(x=hit.point.x-0.5, y=hit.point.y-0.5),
                    Point(x=hit.point.x+0.5, y=hit.point.y-0.5),
                    Point(x=hit.point.x+0.5, y=hit.point.y+0.5),
                    Point(x=hit.point.x-0.5, y=hit.point.y+0.5),
                ])
                obstacles.update()
                return
            if object_type == 'obstacle' and click_mode.value == 'obstacles':
                del world.obstacles[hit.object.name.split('_')[1]]
                hit.object.delete()
                return

    with ui.scene(640, 480, on_click=handle_click) as scene:
        rosys.ui.robot_object(debug=True)
        obstacles = rosys.ui.obstacle_object(world.obstacles)
        path3d = rosys.ui.path_object()

    with ui.row():
        rosys.ui.automation_controls()
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')


ui.run(title='obstacles', port=8080)
