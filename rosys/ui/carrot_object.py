from nicegui.elements.scene_objects import Sphere
from nicegui.elements.scene_object3d import Object3D
from rosys.world.robot import Robot


class CarrotObject(Object3D):

    def __init__(self, robot: Robot):
        super().__init__('group')
        self.robot = robot
        with self:
            self.sphere = Sphere(0.05)
        self.update()

    def update(self):
        if self.robot.carrot is None:
            self.sphere.material('#ff8800', 0)
        else:
            self.sphere.material('#ff8800', 1)
            self.move(self.robot.carrot.x, self.robot.carrot.y)
