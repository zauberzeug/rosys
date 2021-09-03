#!/usr/bin/env python3
import asyncio
import os
from uuid import uuid4
import starlette
from nicegui import app, ui
from rosys.actors.pathplanning.obstacle_map import ObstacleMap
from rosys.actors.pathplanning.planner import Planner
from rosys.world.obstacle import Obstacle
from rosys.automations.drive_path import drive_path
from rosys.runtime import Runtime
from rosys.ui.joystick import Joystick
from rosys.world.mode import Mode
from rosys.world.point import Point
from rosys.world.robot import Robot
from rosys.world.spline import Spline
from rosys.world.world import World, WorldState
from hardware import hardware
import logging

import icecream
icecream.install()

mode = Mode.REAL if os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0 else Mode.SIMULATION
logging.warning(f'we are using {mode}')
world = World(mode=mode, robot=Robot(hardware=hardware))

runtime = Runtime(world, backup_filepath='/tmp/world.json')

with ui.card():
    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'{world.time:.3f} s ({world.robot.prediction})'))

    click_mode = ui.toggle({'drive': 'Drive', 'plan': 'Plan', 'obstacles': 'Obstacles'}).props('outline clearable')

    def update_path_in_scene():
        [obj.delete() for obj in list(scene.view.objects.values()) if obj.name == 'curve']
        for spline in world.path:
            scene.curve(
                [spline.start.x, spline.start.y, 0],
                [spline.control1.x, spline.control1.y, 0],
                [spline.control2.x, spline.control2.y, 0],
                [spline.end.x, spline.end.y, 0]).material('#ff8800').with_name('curve')

    def update_obstacles_in_scene():
        [obj.delete() for obj in list(scene.view.objects.values()) if (obj.name or '').startswith('obstacle_')]
        for obstacle in world.obstacles.values():
            outline = [[point.x, point.y] for point in obstacle.outline]
            scene.extrusion(outline, 0.1).with_name(f'obstacle_{obstacle.id}')

    def handle_click(msg):
        if msg.click_type != 'dblclick':
            return
        for hit in msg.hits:
            object_type = hit.object_id if hit.object is None else (hit.object.name or '').split('_')[0]
            if object_type == 'ground' and click_mode.value == 'drive':
                start = world.robot.prediction.point
                target = Point(x=hit.point.x, y=hit.point.y)
                world.path = [Spline(
                    start=start,
                    control1=start.interpolate(target, 1/3),
                    control2=start.interpolate(target, 2/3),
                    end=hit.point,
                )]
                update_path_in_scene()
                runtime.automator.replace(drive_path(world, runtime.esp))
                runtime.resume()
                return
            if object_type == 'ground' and click_mode.value == 'plan':
                obstacle_map = ObstacleMap.from_world(world)
                planner = Planner(obstacle_map)
                planner.set_goal([hit.point.x, hit.point.y, 0])
                planner.search([world.robot.prediction.x, world.robot.prediction.y, world.robot.prediction.yaw])
                world.path = [step.spline for step in planner.path]
                update_path_in_scene()
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
                update_obstacles_in_scene()
                return
            if object_type == 'obstacle' and click_mode.value == 'obstacles':
                del world.obstacles[hit.object.name.split('_')[1]]
                hit.object.delete()
                return
    with ui.row():
        with ui.scene(640, 480, on_click=handle_click) as scene:
            outline = list(map(list, world.robot.shape.outline))
            robot = scene.extrusion(outline, world.robot.shape.height).material('#4488ff', 0.5)
            update_obstacles_in_scene()
            ui.timer(0.05, lambda: robot
                     .move(world.robot.prediction.x, world.robot.prediction.y)
                     .rotate(0, 0, world.robot.prediction.yaw) and False)
        Joystick(size=50, color='blue', steerer=runtime.steerer)

    with ui.row():
        ui.button(on_click=runtime.resume).props('outline icon=play_arrow') \
            .bind_visibility_from(world.state, value=WorldState.PAUSED)
        ui.button(on_click=lambda: asyncio.create_task(runtime.pause())).props('outline icon=pause') \
            .bind_visibility_from(world.state, value=WorldState.RUNNING)
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')
        ui.button('clear odrive', on_click=lambda: runtime.esp.send('can request 018')).props('outline')
        ui.button('restart odrive', on_click=lambda: runtime.esp.send('can request 016')).props('outline')

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())

app.routes.insert(0, starlette.routing.Route(
    '/world', lambda *_: starlette.responses.Response(content=world.json(exclude={'image_data'}), media_type='text/json')))

ui.run(title="obstacles")
