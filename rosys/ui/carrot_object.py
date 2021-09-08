from nicegui.elements.scene_objects import Sphere
from nicegui.elements.scene_object3d import Object3D
from rosys.world.robot import Robot


class CarrotObject(Object3D):

    def __init__(self, robot: Robot):
        super().__init__('group')
        self.robot = robot
        with self:
            self.sphere0 = Sphere(0.03)
            self.sphere1 = Sphere(0.05).move(robot.parameters.carrot_offset)
        self.update()

    def update(self):
        if self.robot.carrot is None:
            self.sphere0.material('#ff8800', 0)
            self.sphere1.material('#ff8800', 0)
        else:
            self.sphere0.material('#ff8800', 1)
            self.sphere1.material('#ff8800', 1)
            self.move(self.robot.carrot.x, self.robot.carrot.y)
            self.rotate(0, 0, self.robot.carrot.yaw)
