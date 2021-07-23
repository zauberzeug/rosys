from pydantic import BaseModel
from typing import Callable, Optional
import time
from nicegui.elements.custom_view import CustomView
from nicegui.elements.element import Element
from ..world.pose import Pose
from ..world.robot import RobotShape
from ..world.image import Image
from ..world.camera import Camera
from ..world.spline import Spline


class ThreeView(CustomView):

    def __init__(self, *, on_click: Callable):

        super().__init__('three', __file__, ['three.min.js', 'OrbitControls.js'], elements={})

        self.on_click = on_click
        self.allowed_events = ['onClick']
        self.initialize(temp=False, onClick=self.handle_click)

    def handle_click(self, msg):

        if self.on_click is not None:
            return self.on_click(msg)
        return False


class ThreeElement(BaseModel):

    id: str
    type: str
    pose: Optional[Pose]
    modified: float = 0
    properties: dict = {}


class Three(Element):

    def __init__(self, *, on_click: Callable = None):

        super().__init__(ThreeView(on_click=on_click))

    def set_robot(self, id: str, color: str, pose: Pose, shape: RobotShape):

        element = ThreeElement(id=id, type='robot', pose=pose, properties={'color': color, 'shape': shape.dict()})
        element.pose.time = 0
        element_dict = element.dict()
        if self.view.options.elements.get(id) == element_dict:
            return False
        self.view.options.elements[id] = element_dict

    def update_images(self, images: list[Image], cameras: dict[str, Camera]):

        dirty = False
        new_images = {image.mac: image for image in images}
        new_image_ids = [image.id for image in new_images.values()]
        old_image_ids = [id for id, e in self.view.options.elements.items() if e['type'] == 'image']
        for id in old_image_ids:
            if id not in new_image_ids:
                del self.view.options.elements[id]
                dirty = True
        for image in new_images.values():
            if image.id not in self.view.options.elements:
                camera = cameras.get(image.mac)
                if camera is not None and camera.projection is not None:
                    properties = image.dict() | {'camera': camera.dict()}
                    jp_element = ThreeElement(id=image.id, type='image', properties=properties)
                    self.view.options.elements[image.id] = jp_element.dict()
                    dirty = True
        return dirty

    def update_path(self, path: list[Spline]):

        id = 'path'
        properties = {'splines': [spline.dict() for spline in path]}
        element = ThreeElement(id=id, type='path', properties=properties)
        jp_element = self.view.options.elements.get(id)
        if jp_element is not None and jp_element['properties'] == properties:
            return False
        element.modified = time.time()
        self.view.options.elements[id] = element.dict()

    def update_carrot(self, pose: Pose):

        id = 'carrot'
        element = ThreeElement(id=id, type='carrot', pose=pose)
        if element.pose is not None:
            element.pose.time = 0
        element_dict = element.dict()
        if self.view.options.elements.get(id) == element_dict:
            return False
        self.view.options.elements[id] = element_dict

    def update_cameras(self, cameras: dict[str, Camera]):

        dirty = False
        old_macs = [id for id, e in self.view.options.elements.items() if e['type'] == 'camera']
        for mac in old_macs:
            if mac not in cameras:
                del self.view.options.elements[mac]
                dirty = True
        for mac, camera in cameras.items():
            if mac not in self.view.options.elements and camera.calibration is not None:
                jp_element = ThreeElement(id=mac, type='camera', properties=camera.calibration.dict())
                self.view.options.elements[mac] = jp_element.dict()
                dirty = True
        return dirty
