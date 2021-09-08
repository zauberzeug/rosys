from nicegui.elements.scene_objects import Extrusion, Sphere
from nicegui.elements.scene_object3d import Object3D
from rosys.world.robot import Robot


class RobotObject(Object3D):

    def __init__(self, robot: Robot, *, show_hook: bool = False, wireframe: bool = False):
        super().__init__('group')
        self.robot = robot
        with self:
            outline = list(map(list, robot.shape.outline))
            Extrusion(outline, robot.shape.height, wireframe=wireframe).material('#4488ff', 0.5)
            if show_hook:
                Sphere(0.03).material('#4488ff', 0.5)
                Sphere(0.05).material('#4488ff', 0.5).move(robot.parameters.hook_offset)
        self.update()

    def update(self):
        self.move(self.robot.prediction.x, self.robot.prediction.y)
        self.rotate(0, 0, self.robot.prediction.yaw)
