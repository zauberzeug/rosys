from typing import Optional

from nicegui.elements.scene_objects import Group

from .area_manipulation import AreaManipulation, AreaManipulationMode
from .path_planner import PathPlanner

HEIGHT = 0.1
DIAMETER = 0.2


class AreaObject(Group):

    def __init__(self, path_planner: PathPlanner, area_manipulation: Optional[AreaManipulation] = None) -> None:
        super().__init__()

        self.path_planner = path_planner
        self.area_manipulation = area_manipulation

        path_planner.AREAS_CHANGED.register_ui(lambda _: self.update())
        if area_manipulation:
            area_manipulation.MODE_CHANGED.register_ui(lambda _: self.update())

        self.update()

    def update(self) -> None:
        [obj.delete() for obj in list(self.scene.objects.values()) if (obj.name or '').startswith('area_')]
        for area in self.path_planner.areas.values():
            if area.closed:
                outline = [[point.x, point.y] for point in area.outline]
            else:
                outline = [[point.x, point.y] for point in area.outline[:-1] + area.outline[-1:0:-1]]
                last_edge = [[area.outline[0].x, area.outline[0].y], [area.outline[-1].x, area.outline[-1].y]]
                self.scene.extrusion(last_edge, HEIGHT, wireframe=True) \
                    .with_name(f'area_{area.id}_last_edge').material(area.color, 0.2)
            self.scene.extrusion(outline, HEIGHT, wireframe=True).with_name(f'area_{area.id}').material(area.color)

            if self.area_manipulation and self.area_manipulation.mode == AreaManipulationMode.EDIT:
                for p, point in enumerate(area.outline):
                    self.scene.sphere(DIAMETER / 2).move(point.x, point.y, HEIGHT / 2) \
                        .with_name(f'area_{area.id}_{p}').material(area.color).draggable()
