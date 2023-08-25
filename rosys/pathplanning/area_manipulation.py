import uuid
from enum import Enum
from typing import Optional

from .. import rosys
from ..geometry import LineSegment, Point
from .area import Area
from .path_planner import PathPlanner


class AreaManipulationCommand(Enum):
    NONE = None
    ADD = 'Add'
    EDIT = 'Edit'
    DELETE = 'Delete'


class AreaManipulation:

    def __init__(self, path_planner: PathPlanner) -> None:
        self.path_planner = path_planner
        self.active_area: Optional[Area] = None
        self.command = AreaManipulationCommand.NONE

    @property
    def can_undo(self) -> bool:
        return self.active_area is not None and len(self.active_area.outline) > 0

    def add_point(self, point: Point) -> None:
        if self.command != AreaManipulationCommand.ADD:
            return

        if self.active_area is None:
            area = Area(id=str(uuid.uuid4()), outline=[], closed=False)
            self.path_planner.areas[area.id] = area
            self.active_area = area

        if self._would_cause_self_intersection(point, self.active_area.outline):
            rosys.notify('Edges must not intersect!', type='negative')
            return

        self.active_area.outline.append(point)
        self._emit_change_event()

    def undo(self) -> None:
        if not self.can_undo:
            return
        assert self.active_area is not None
        self.active_area.outline.pop()
        self._emit_change_event()

    def cancel(self) -> None:
        if self.active_area:
            self.path_planner.areas.pop(self.active_area.id)
            self._emit_change_event()
        self.active_area = None
        self.command = AreaManipulationCommand.NONE

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
        self.command = AreaManipulationCommand.NONE

    def clear_all(self) -> None:
        self.path_planner.areas.clear()
        self.active_area = None
        self.command = AreaManipulationCommand.NONE
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
