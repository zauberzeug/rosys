import numpy as np
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Extrusion
from ..world import Area


class AreaObject(Object3D):

    def __init__(self, areas: dict[str, Area]):
        super().__init__('group')
        self.areas = areas
        self.update()

    def update(self) -> bool:
        [obj.delete() for obj in list(self.view.objects.values()) if (obj.name or '').startswith('area_')]
        for area in self.areas.values():
            if len(area.outline) == 1:
                outline = [[area.outline[0].x + 0.05 * np.cos(phi), area.outline[0].y + 0.05 * np.sin(phi)]
                           for phi in np.linspace(0, 2 * np.pi, 16, endpoint=False)]
            else:
                outline = [[point.x, point.y] for point in area.outline]
            Extrusion(outline, 0.1, wireframe=True).with_name(f'area_{area.id}').material('green')
        return False  # NOTE: avoid JustPy page_update
