from typing import Callable, List
from nicegui.elements.custom_view import CustomView
from nicegui.elements.element import Element
from ..world.pose import Pose
from ..world.image import Image


class ThreeView(CustomView):

    def __init__(self, *, robot_pose: Pose, follow_robot: bool, on_click: Callable):

        super().__init__('three', __file__, [
            'https://cdn.jsdelivr.net/npm/three@0.129.0/build/three.min.js',
            'https://cdn.jsdelivr.net/npm/three@0.129.0/examples/js/controls/OrbitControls.js',
        ], robot_pose=robot_pose.dict(), follow_robot=follow_robot, images=[])

        self.on_click = on_click
        self.allowed_events = ['onClick']
        self.initialize(temp=False, onClick=self.handle_click)

    def handle_click(self, msg):

        if self.on_click is not None:
            self.on_click(msg)


class Three(Element):

    def __init__(self, robot_pose: Pose, *, follow_robot: bool = True, on_click: Callable = None):

        super().__init__(ThreeView(robot_pose=robot_pose, follow_robot=follow_robot, on_click=on_click))

    def set_robot_pose(self, pose: Pose):

        new_pose = pose.dict()
        if self.view.options.robot_pose == new_pose:
            return False
        self.view.options.robot_pose = new_pose

    def update_images(self, images: List[Image]):

        self.view.options.images = [image.dict() for image in images]
