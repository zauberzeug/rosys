import logging
import json
import os
from .world import Obstacle, RobotParameters, World


class Persistence:

    def __init__(self, world: World, filepath: str = '~/.rosys/world.json'):
        self.world = world
        self.filepath = os.path.expanduser(filepath)

    def dump(self) -> dict:
        return {
            'robot': {'parameters': self.world.robot.parameters.dict()},
            'obstacles': {id: obstacle.dict() for id, obstacle in self.world.obstacles.items()},
        }

    def load(self, dict: dict):
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
            logging.exception(f'Could not load from backup at {self.filepath}')
