from nicegui.elements.scene_objects import Group, Object3D

from .area import Area
from .area_manipulation import AreaManipulation, AreaManipulationMode
from .path_planner import PathPlanner

HEIGHT = 0.1
DIAMETER = 0.2


class AreaObject(Group):

    def __init__(self, path_planner: PathPlanner, area_manipulation: AreaManipulation | None = None) -> None:
        super().__init__()

        self.path_planner = path_planner
        self.area_manipulation = area_manipulation

        path_planner.AREAS_CHANGED.register_ui(self.update)
        if area_manipulation:
            area_manipulation.MODE_CHANGED.register_ui(lambda _: self.update())

        self.update()

    def update(self, areas: list[Area] | None = None) -> None:
        area_ids = [area.id for area in areas] if areas else []

        def should_remove(obj: Object3D) -> bool:
            words = (obj.name or '').split('_')
            return (words[0] == 'area') and not (area_ids and words[1] not in area_ids)
        self.scene.delete_objects(should_remove)

        for area in self.path_planner.areas.values():
            if area_ids and area.id not in area_ids:
                continue

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
                        .with_name(f'area_{area.id}_corner_{p}').material(area.color).draggable()
                    next_point = area.outline[(p + 1) % len(area.outline)]
                    mid_point = point.interpolate(next_point, 0.5)
                    self.scene.sphere(DIAMETER / 2).move(mid_point.x, mid_point.y, HEIGHT / 2) \
                        .with_name(f'area_{area.id}_mid_{p}').material(area.color, 0.2).draggable()
