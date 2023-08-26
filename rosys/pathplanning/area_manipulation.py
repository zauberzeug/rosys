import uuid
from enum import Enum
from typing import Optional

from nicegui import binding, ui
from nicegui.events import SceneClickEventArguments, SceneDragEventArguments

from .. import rosys
from ..event import Event
from ..geometry import LineSegment, Point
from .area import Area
from .path_planner import PathPlanner


class AreaManipulationMode(Enum):
    IDLE = None
    EDIT = 'Edit'
    DELETE = 'Delete'

# TODO
# - create areas of different types/colors
# - allow adjusting the sphere size
# - delete area on double click
# - translation


class AreaManipulation:
    mode = binding.BindableProperty(on_change=lambda sender, value: sender.MODE_CHANGED.emit(value))

    def __init__(self, path_planner: PathPlanner) -> None:
        self.path_planner = path_planner
        self.active_area: Optional[Area] = None
        self.mode = AreaManipulationMode.IDLE

        self.MODE_CHANGED = Event()
        """the mode has changed (argument: new mode)"""

    def create_ui(self) -> ui.row:
        with ui.row() as row:
            area_toggle = ui.toggle({
                AreaManipulationMode.EDIT: 'Edit',
                AreaManipulationMode.DELETE: 'Delete',
            }).props('outline').bind_value(self, 'mode')
            ui.button(icon='close', on_click=self.cancel) \
                .props('outline') \
                .bind_visibility_from(area_toggle, 'value', bool)
            ui.button(icon='done', on_click=self.done) \
                .props('outline') \
                .bind_visibility_from(area_toggle, 'value', bool)
            ui.button(icon='undo', on_click=self.undo) \
                .props('outline') \
                .bind_visibility_from(self, 'can_undo')
            ui.button('Clear all', on_click=self.clear_all) \
                .props('outline color=red') \
                .bind_visibility_from(area_toggle, 'value', value=AreaManipulationMode.DELETE)
        return row

    @property
    def can_undo(self) -> bool:
        return self.active_area is not None and len(self.active_area.outline) > 0

    def add_point(self, point: Point) -> None:
        if self.active_area is None:
            area = Area(id=str(uuid.uuid4()), outline=[], closed=False)
            self.path_planner.areas[area.id] = area
            self.active_area = area

        if self._would_cause_self_intersection(point, self.active_area.outline):
            rosys.notify('Edges must not intersect!', type='negative')
            return

        self.active_area.outline.append(point)
        self._emit_change_event()

    def delete_area(self, point: Point) -> None:
        for area in self.path_planner.areas.values():
            if area.contains(point):
                self.path_planner.areas.pop(area.id)
                self._emit_change_event()
                return

    def undo(self) -> None:
        if not self.can_undo:
            return
        assert self.active_area is not None
        self.active_area.outline.pop()
        if not self.active_area.outline:
            self.path_planner.areas.pop(self.active_area.id)
            self.active_area = None
        self._emit_change_event()

    def cancel(self) -> None:
        if self.active_area:
            self.path_planner.areas.pop(self.active_area.id)
            self._emit_change_event()
        self.active_area = None
        self.mode = AreaManipulationMode.IDLE

    def done(self) -> None:
        if self.active_area:
            if len(self.active_area.outline) < 3:
                rosys.notify('Areas must have at least 3 points!', type='negative')
                return
            if self._would_cause_self_intersection(self.active_area.outline[0], self.active_area.outline[1:]):
                rosys.notify('Edges must not intersect!', type='negative')
                return
            self.active_area.closed = True
            self._emit_change_event()
        self.active_area = None
        self.mode = AreaManipulationMode.IDLE

    def clear_all(self) -> None:
        self.path_planner.areas.clear()
        self.active_area = None
        self.mode = AreaManipulationMode.IDLE
        self._emit_change_event()

    def handle_click(self, e: SceneClickEventArguments) -> None:
        if e.click_type == 'dblclick':
            for hit in e.hits:
                if hit.object_id == 'ground':
                    target = Point(x=hit.x, y=hit.y)
                    if self.mode == AreaManipulationMode.EDIT:
                        self.add_point(target)
                    elif self.mode == AreaManipulationMode.DELETE:
                        self.delete_area(target)

    def handle_drag_end(self, e: SceneDragEventArguments) -> None:
        words = e.object_name.split('_')
        area_id = words[1]
        point_index = int(words[2])
        assert area_id in self.path_planner.areas
        area = self.path_planner.areas[area_id]
        area.outline[point_index].x = e.x
        area.outline[point_index].y = e.y
        self._emit_change_event()

    def _emit_change_event(self) -> None:
        self.path_planner.AREAS_CHANGED.emit(self.path_planner.areas)
        self.path_planner.invalidate()

    def _would_cause_self_intersection(self, point: Point, polyline: list[Point]) -> bool:
        if polyline:
            new_edge = LineSegment(point1=polyline[-1], point2=point)
            for i in range(len(polyline) - 2):
                edge = LineSegment(point1=polyline[i], point2=polyline[i + 1])
                if edge.intersect(new_edge):
                    return True
        return False
