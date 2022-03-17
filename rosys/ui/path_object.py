from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Curve
from ..world import PathSegment


class PathObject(Object3D):

    def __init__(self):
        super().__init__('group')

    def update(self, path: list[PathSegment], height: float = 0) -> bool:
        [obj.delete() for obj in list(self.view.objects.values()) if obj.name == 'path']
        for segment in path:
            Curve(
                [segment.spline.start.x, segment.spline.start.y, height],
                [segment.spline.control1.x, segment.spline.control1.y, height],
                [segment.spline.control2.x, segment.spline.control2.y, height],
                [segment.spline.end.x, segment.spline.end.y, height],
            ).material('#ff8800').with_name('path')
        return False  # NOTE: avoid JustPy page_update
