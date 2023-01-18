from nicegui.elements.scene_objects import Extrusion, Group

from .obstacle import Obstacle
from .path_planner import PathPlanner


class ObstacleObject(Group):

    def __init__(self, path_planner: PathPlanner):
        super().__init__()

        self.update(path_planner.obstacles)
        path_planner.OBSTACLES_CHANGED.register_ui(self.update)

    def update(self, obstacles: dict[str, Obstacle]) -> None:
        [obj.delete() for obj in list(self.scene.objects.values()) if (obj.name or '').startswith('obstacle_')]
        with self.scene:
            for obstacle in obstacles.values():
                outline = [[point.x, point.y] for point in obstacle.outline]
                Extrusion(outline, 0.1).with_name(f'obstacle_{obstacle.id}')
