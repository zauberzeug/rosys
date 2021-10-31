#!/usr/bin/env python3
import asyncio
import os
from uuid import uuid4
import starlette
from nicegui import app, ui
import rosys
import rosys.ui
from rosys.actors.pathplanning.planner import Planner
from rosys.automations import drive_path
from rosys.ui.obstacle_object import ObstacleObject
from rosys.ui.path_object import PathObject
from rosys.world.obstacle import Obstacle
from rosys.world.path_segment import PathSegment
from rosys.world.point import Point
from rosys.world.pose import Pose
from rosys.world.spline import Spline
from rosys.world.world import World, WorldState
from hardware import hardware

import icecream
icecream.install()

shape = rosys.RobotShape(outline=[(0, 0), (-0.5, -0.5), (1.5, -0.5), (1.75, 0), (1.5, 0.5), (-0.5, 0.5)])
world = World(robot=rosys.Robot(hardware=hardware, shape=shape))
planner = Planner(world)
runtime = rosys.Runtime(world, rosys.Persistence(world, '~/.rosys/obstacles/world.json'))
rosys.ui.configure(ui, runtime)

rosys.ui.keyboard_control()
with ui.card():
    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'{world.time:.3f} s ({world.robot.prediction})'))

    click_mode = ui.toggle({'drive': 'Drive', 'plan': 'Plan', 'obstacles': 'Obstacles'}).props('outline clearable')

    def handle_click(msg):
        if msg.click_type != 'dblclick':
            return
        for hit in msg.hits:
            object_type = hit.object_id if hit.object is None else (hit.object.name or '').split('_')[0]
            if object_type == 'ground' and click_mode.value == 'drive':
                start = world.robot.prediction.point
                target = Point(x=hit.point.x, y=hit.point.y)
                world.path[:] = [PathSegment(spline=Spline(
                    start=start,
                    control1=start.interpolate(target, 1/3),
                    control2=start.interpolate(target, 2/3),
                    end=hit.point,
                ))]
                path.update()
                runtime.automator.replace(drive_path(world, runtime.esp))
                runtime.resume()
                return
            if object_type == 'ground' and click_mode.value == 'plan':
                target_yaw = world.robot.prediction.point.direction(hit.point)
                planner.search(goal=Pose(x=hit.point.x, y=hit.point.y, yaw=target_yaw), timeout=3.0)
                world.path[:] = [PathSegment(spline=step.spline, backward=step.backward) for step in planner.path]
                path.update()
                runtime.automator.replace(drive_path(world, runtime.esp))
                return
            if object_type == 'ground' and click_mode.value == 'obstacles':
                id = str(uuid4())
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
        robot = rosys.ui.robot_object(debug=True)
        obstacles = ObstacleObject(world.obstacles)
        path = rosys.ui.path_object()
        ui.timer(0.05, robot.update)

    with ui.row():
        ui.button(on_click=runtime.resume).props('outline icon=play_arrow') \
            .bind_visibility_from(world, 'state', value=WorldState.PAUSED)
        ui.button(on_click=lambda: asyncio.create_task(runtime.pause())).props('outline icon=pause') \
            .bind_visibility_from(world, 'state', value=WorldState.RUNNING)
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')
        ui.button('clear odrive', on_click=lambda: runtime.esp.send('can request 018')).props('outline')
        ui.button('restart odrive', on_click=lambda: runtime.esp.send('can request 016')).props('outline')

# TODO replace with new NiceGUI api
app.routes.insert(0, starlette.routing.Route(
    '/world', lambda *_:
    starlette.responses.Response(content=world.json(exclude={'image_data'}), media_type='text/json')))

ui.run(title="obstacles", port=8080)
