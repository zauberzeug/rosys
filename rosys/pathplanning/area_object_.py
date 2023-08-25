import numpy as np
from nicegui.elements.scene_objects import Extrusion, Group

from .area import Area
from .path_planner import PathPlanner


class AreaObject(Group):

    def __init__(self, path_planner: PathPlanner) -> None:
        super().__init__()

        self.update(path_planner.areas)
        path_planner.AREAS_CHANGED.register_ui(self.update)

    def update(self, areas: dict[str, Area]) -> None:
        [obj.delete() for obj in list(self.scene.objects.values()) if (obj.name or '').startswith('area_')]
        for area in areas.values():
            if len(area.outline) == 1:
                outline = [[area.outline[0].x + 0.05 * np.cos(phi), area.outline[0].y + 0.05 * np.sin(phi)]
                           for phi in np.linspace(0, 2 * np.pi, 16, endpoint=False)]
            elif area.closed:
                outline = [[point.x, point.y] for point in area.outline]
            else:
                outline = [[point.x, point.y] for point in area.outline[:-1] + area.outline[-1:0:-1]]
            with self.scene:
                Extrusion(outline, 0.1, wireframe=True).with_name(f'area_{area.id}').material(area.color)
