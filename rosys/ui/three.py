from typing import Callable
import base64
from nicegui.elements.custom_view import CustomView
from nicegui.elements.element import Element
from ..world.pose import Pose
from ..world.image import Image
from ..world.camera import Camera


class ThreeView(CustomView):

    def __init__(self, *, robot_pose: Pose, follow_robot: bool, on_click: Callable):

        super().__init__('three', __file__, [
            'https://cdn.jsdelivr.net/npm/three@0.129.0/build/three.min.js',
            'https://cdn.jsdelivr.net/npm/three@0.129.0/examples/js/controls/OrbitControls.js',
        ], robot_pose=robot_pose.dict(), follow_robot=follow_robot, add_images=[], remove_images=[])

        self.on_click = on_click
        self.allowed_events = ['onClick', 'imagesUpdated']
        self.initialize(temp=False, onClick=self.handle_click, imagesUpdated=self.handle_images_updated)
        self.client_image_ids = []
        self.transmitting = False

    def handle_click(self, msg):

        if self.on_click is not None:
            self.on_click(msg)

    def handle_images_updated(self, msg):

        print('end transmitting', msg.image_ids, flush=True)
        self.options.add_images = []
        self.options.remove_images = []
        self.client_image_ids = msg.image_ids
        self.transmitting = False


class Three(Element):

    def __init__(self, robot_pose: Pose, *, follow_robot: bool = True, on_click: Callable = None):

        super().__init__(ThreeView(robot_pose=robot_pose, follow_robot=follow_robot, on_click=on_click))

    def is_transmitting(self):

        return self.view.options.add_images or self.view.options.remove_images

    def set_robot_pose(self, pose: Pose):

        if self.view.transmitting:
            return False  # NOTE: avoid updates to view options while images are transmitted

        new_pose = pose.dict()
        if self.view.options.robot_pose == new_pose:
            return False
        self.view.options.robot_pose = new_pose
        return False

    def update_images(self, images: list[Image], image_data: dict[str, bytes], cameras: dict[str, Camera]):

        if self.view.transmitting:
            print('still transmitting', flush=True)
            return False

        latest_images = {
            image.mac: image
            for image in images
            if image.id in image_data and image.mac in cameras and cameras[image.mac].projection is not None
        }
        latest_image_ids = [image.id for image in latest_images.values()]

        for image in latest_images.values():
            if image.id not in self.view.client_image_ids:
                self.view.options.add_images.append(image.dict() | {
                    'data': 'data:image/jpeg;base64,' + base64.b64encode(image_data[image.id]).decode("utf-8"),
                    'camera': cameras[image.mac].dict(),
                })

        for image_id in self.view.client_image_ids:
            if image_id not in latest_image_ids:
                self.view.options.remove_images.append(image_id)

        self.view.transmitting = any(self.view.options.add_images) or any(self.view.options.remove_images)
        if self.view.transmitting:
            print('start transmitting...', latest_image_ids, flush=True)

        return False
