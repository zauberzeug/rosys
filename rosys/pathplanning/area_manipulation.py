import uuid
from enum import Enum

import numpy as np
from nicegui import binding, ui
from nicegui.events import SceneClickEventArguments, SceneDragEventArguments

from .. import rosys
from ..event import Event
from ..geometry import Point
from .area import Area
from .path_planner import PathPlanner


class AreaManipulationMode(Enum):
    IDLE = None
    EDIT = 'Edit'
    DELETE = 'Delete'


class Translator:
    Edit = 'Edit'
    Delete = 'Delete'
    Cancel = 'Cancel'
    Done = 'Done'
    Remove_last_point = 'Remove last point'
    Edges_must_not_intersect = 'Edges must not intersect'
    Areas_must_have_at_least_3_points = 'Areas must have at least 3 points'


class AreaManipulation:
    mode = binding.BindableProperty(on_change=lambda sender, value: sender.MODE_CHANGED.emit(value))
    area_type = binding.BindableProperty()
    area_color = binding.BindableProperty()

    def __init__(self, path_planner: PathPlanner) -> None:
        self.path_planner = path_planner
        self.active_area: Area | None = None
        self.mode = AreaManipulationMode.IDLE
        self.area_type = None
        self.area_color = 'green'
        self.translator: Translator = rosys.translator or Translator()

        self.MODE_CHANGED = Event[AreaManipulationMode]()
        """the mode has changed (argument: new mode)"""

    def create_ui(self) -> ui.row:
        t = self.translator
        with ui.row() as row:
            area_toggle = ui.toggle({
                AreaManipulationMode.EDIT: t.Edit,
                AreaManipulationMode.DELETE: t.Delete,
            }).props('outline').bind_value(self, 'mode')
            ui.button(icon='close', on_click=self.cancel) \
                .props('outline') \
                .tooltip(t.Cancel) \
                .bind_visibility_from(area_toggle, 'value', value=AreaManipulationMode.EDIT)
            ui.button(icon='done', on_click=self.done) \
                .props('outline') \
                .tooltip(t.Done) \
                .bind_visibility_from(area_toggle, 'value', bool)
            ui.button(icon='undo', on_click=self.undo) \
                .props('outline') \
                .tooltip(t.Remove_last_point) \
                .bind_visibility_from(self, 'can_undo')
        return row

    @property
    def can_undo(self) -> bool:
        return self.active_area is not None and len(self.active_area.outline) > 0

    def add_point(self, point: Point) -> None:
        if self.active_area is None:
            area = Area(id=str(uuid.uuid4()), outline=[], closed=False, type=self.area_type, color=self.area_color)
            self.path_planner.areas[area.id] = area
            self.active_area = area

        if self.active_area.would_cause_self_intersection(point):
            rosys.notify(self.translator.Edges_must_not_intersect, type='negative')
            return

        self.active_area.outline.append(point)
        self._emit_change_event(self.active_area)

    def delete_area(self, point: Point) -> None:
        for area in self.path_planner.areas.values():
            if area.contains(point):
                self.path_planner.areas.pop(area.id)
                self.active_area = None
                self._emit_change_event(area)
                return
        if not self.path_planner.areas:
            self.mode = AreaManipulationMode.IDLE

    def delete_point(self, area: Area, point_index: int) -> None:
        del area.outline[point_index]
        if not area.outline:
            self.path_planner.areas.pop(area.id)
            self.active_area = None
        self._emit_change_event(area)

    def undo(self) -> None:
        if not self.can_undo:
            return
        assert self.active_area is not None
        self.active_area.outline.pop()
        if not self.active_area.outline:
            self.path_planner.areas.pop(self.active_area.id)
            self.active_area = None
        self._emit_change_event(self.active_area)

    def cancel(self) -> None:
        if self.active_area:
            self.path_planner.areas.pop(self.active_area.id)
            self._emit_change_event(self.active_area)
        self.active_area = None
        self.mode = AreaManipulationMode.IDLE

    def try_close_active_area(self) -> bool:
        if self.active_area:
            if len(self.active_area.outline) < 3:
                rosys.notify(self.translator.Areas_must_have_at_least_3_points, type='negative')
                return False
            if self.active_area.would_cause_self_intersection(self.active_area.outline[0]):
                rosys.notify(self.translator.Edges_must_not_intersect, type='negative')
                return False
            self.active_area.closed = True
            self._emit_change_event(self.active_area)
        self.active_area = None
        return True

    def done(self) -> None:
        if self.try_close_active_area():
            self.mode = AreaManipulationMode.IDLE

    def handle_click(self, e: SceneClickEventArguments) -> None:
        if e.click_type == 'dblclick':
            for hit in e.hits:
                target = Point(x=hit.x, y=hit.y)
                if hit.object_id == 'ground':
                    if self.mode == AreaManipulationMode.EDIT:
                        self.add_point(target)
                    elif self.mode == AreaManipulationMode.DELETE:
                        self.delete_area(target)
                    return
                if (hit.object_name or '').startswith('area_') and '_corner_' in hit.object_name:
                    area_id = hit.object_name.split('_')[1]
                    area = self.path_planner.areas[area_id]
                    distances = [point.distance(target) for point in area.outline]
                    point_index = int(np.argmin(distances))
                    if area is self.active_area and point_index == 0:
                        self.try_close_active_area()
                    else:
                        self.delete_point(area, point_index)
                    return

    def handle_drag_end(self, e: SceneDragEventArguments) -> None:
        _, area_id, type_, point_index_str = e.object_name.split('_')
        area = self.path_planner.areas[area_id]
        point_index = int(point_index_str)
        new_point = Point(x=e.x, y=e.y)
        if type_ == 'corner':
            old_point = area.outline.pop(point_index)
            if area.would_cause_self_intersection(new_point, point_index):
                rosys.notify(self.translator.Edges_must_not_intersect, type='negative')
                area.outline.insert(point_index, old_point)
            else:
                area.outline.insert(point_index, new_point)
        elif type_ == 'mid':
            new_index = point_index + 1
            if area.would_cause_self_intersection(new_point, new_index):
                rosys.notify(self.translator.Edges_must_not_intersect, type='negative')
            else:
                area.outline.insert(new_index, new_point)
        self._emit_change_event(None)  # TODO: pass area when https://github.com/zauberzeug/nicegui/issues/1505 is fixed

    def _emit_change_event(self, area: Area | None = None) -> None:
        self.path_planner.AREAS_CHANGED.emit([area] if area else None)
        self.path_planner.request_backup()
