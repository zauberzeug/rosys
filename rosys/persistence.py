import logging
import json
import os
from .world.obstacle import Obstacle
from .world.path_segment import PathSegment
from .world.robot import RobotParameters
from .world.world import World


class Persistence:

    def __init__(self, world: World, filepath: str = '~/.rosys/world.json'):
        self.world = world
        self.filepath = os.path.expanduser(filepath)

    def dump(self) -> dict:
        return {
            'path': [path_segment.dict() for path_segment in self.world.path],
            'robot': {'parameters': self.world.robot.parameters.dict()},
            'obstacles': {id: obstacle.dict() for id, obstacle in self.world.obstacles.items()},
        }

    def load(self, dict: dict):
        self.world.path = [PathSegment.parse_obj(path_segment) for path_segment in dict['path']]
        self.world.robot.parameters = RobotParameters.parse_obj(dict['robot']['parameters'])
        self.world.obstacles = {id: Obstacle.parse_obj(obstacle) for id, obstacle in dict['obstacles'].items()}

    def backup(self):
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        with open(self.filepath, 'w') as f:
            json.dump(self.dump(), f)

    def restore(self):
        if not os.path.exists(self.filepath):
            logging.warning('No backup file found.')
            return
        try:
            with open(self.filepath, 'r') as f:
                self.load(json.load(f))
        except:
            logging.exception('Could not load from backup')
