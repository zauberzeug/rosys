from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Curve
from rosys.world.world import World


class PathObject(Object3D):

    # these will be set by rosys.ui.configure
    world: World = None

    def __init__(self):
        super().__init__('group')
        self.update()

    def update(self) -> bool:
        [obj.delete() for obj in list(self.view.objects.values()) if obj.name == 'path']
        for segment in self.world.path:
            Curve(
                [segment.spline.start.x, segment.spline.start.y, 0],
                [segment.spline.control1.x, segment.spline.control1.y, 0],
                [segment.spline.control2.x, segment.spline.control2.y, 0],
                [segment.spline.end.x, segment.spline.end.y, 0],
            ).material('#ff8800').with_name('path')
        return False  # NOTE: avoid JustPy page_update
