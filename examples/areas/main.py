#!/usr/bin/env python3
from nicegui import ui
from nicegui.events import SceneClickEventArguments

from rosys.geometry import Point, Prism
from rosys.pathplanning import AreaManipulation, AreaManipulationCommand, PathPlanner, area_object

# setup
path_planner = PathPlanner(Prism.default_robot_shape())
area_manipulation = AreaManipulation(path_planner)

# ui
with ui.card():
    with ui.row():
        area_toggle = ui.toggle({
            AreaManipulationCommand.ADD: 'Add',
            AreaManipulationCommand.EDIT: 'Edit',
            AreaManipulationCommand.DELETE: 'Delete',
        }).props('outline').bind_value(area_manipulation, 'command')
        ui.button(icon='close', on_click=area_manipulation.cancel) \
            .props('outline') \
            .bind_visibility_from(area_toggle, 'value', bool)
        ui.button(icon='done', on_click=area_manipulation.done) \
            .props('outline') \
            .bind_visibility_from(area_toggle, 'value', bool)
        ui.button(icon='undo', on_click=area_manipulation.undo) \
            .props('outline') \
            .bind_visibility_from(area_manipulation, 'can_undo')
        ui.button('Clear all', on_click=area_manipulation.clear_all) \
            .props('outline color=red') \
            .bind_visibility_from(area_toggle, 'value', value=AreaManipulationCommand.DELETE)

    def handle_click(e: SceneClickEventArguments):
        if e.click_type == 'dblclick':
            for hit in e.hits:
                if hit.object_id == 'ground':
                    area_manipulation.add_point(Point(x=hit.x, y=hit.y))

    with ui.scene(640, 480, on_click=handle_click) as scene:
        area_object(path_planner)

# start
ui.run(title='areas')
