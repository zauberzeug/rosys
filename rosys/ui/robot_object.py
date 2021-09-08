from nicegui.elements.scene_objects import Extrusion
from nicegui.elements.scene_object3d import Object3D
from rosys.world.robot import Robot


class RobotObject(Object3D):

    def __init__(self, robot: Robot):
        super().__init__('group')
        self.robot = robot
        with self:
            outline = list(map(list, robot.shape.outline))
            robot = Extrusion(outline, robot.shape.height).material('#4488ff', 0.5)
        self.update()

    def update(self):
        self.move(self.robot.prediction.x, self.robot.prediction.y)
        self.rotate(0, 0, self.robot.prediction.yaw)
