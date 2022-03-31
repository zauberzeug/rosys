import json
import logging
import os

from .world import World


class Persistence:

    def __init__(self, world: World, filepath: str = '~/.rosys/world.json'):
        self.world = world
        self.filepath = os.path.expanduser(filepath)
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        self.log = logging.getLogger('rosys.Persistence')

    def dump(self) -> dict:
        robot_excludes = {
            'velocity',
            'shape',
            'parameters',
            'prediction',
            'detection',
            'simulation',
            'odometry',
            'current_velocity',
            'last_movement',
            'hardware_time',
            'clock_offset',
            'carrot',
        }
        return {
            'robot': self.world.robot.dict(exclude=robot_excludes),
            'areas': {id: area.dict() for id, area in self.world.areas.items()},
            'obstacles': {id: obstacle.dict() for id, obstacle in self.world.obstacles.items()},
            'usb_cameras': {id: camera.dict() for id, camera in self.world.usb_cameras.items()},
        }

    def load(self, world: World):
        self.world.robot = world.robot
        self.world.areas = world.areas
        self.world.obstacles = world.obstacles
        self.world.usb_cameras = world.usb_cameras

    def backup(self):
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        try:
            with open(self.filepath, 'w') as f:
                json.dump(self.dump(), f)
        except:
            self.log.exception('failed to backup')

    def parse_world(self, obj: dict) -> World:
        return self.world.parse_obj(obj)  # NOTE we use the provided world for parsing to load all attributes

    def restore(self):
        if not os.path.exists(self.filepath):
            logging.warning('No backup file found.')
            return
        try:
            with open(self.filepath, 'r') as f:
                self.load(self.parse_world(json.load(f)))
        except:
            logging.exception(f'Could not load from backup at {self.filepath}')
