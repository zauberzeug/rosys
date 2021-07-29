import logging
import json
import os
from .world.world import World
from .world.camera import Camera
from .world.robot import RobotParameters
from .world.spline import Spline
from .world.link import Link

filepath = '/data/backup/world.json'


def backup(world: World):

    exclude = {'projection', 'synchronization'}
    dict_ = {
        'cameras': {mac: camera.dict(exclude=exclude) for mac, camera in world.cameras.items()},
        'path': [spline.dict() for spline in world.path],
        'robot': {'parameters': world.robot.parameters.dict()},
        'links': [link.dict() for link in world.links],
    }
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'w') as f:
        json.dump(dict_, f)


def restore(world: World):

    if not os.path.exists(filepath):
        logging.warning('No backup file found.')
        return

    try:
        with open(filepath, 'r') as f:
            dict_ = json.load(f)
        world.cameras = {mac: Camera.parse_obj(camera) for mac, camera in dict_['cameras'].items()}
        world.path = [Spline.parse_obj(spline) for spline in dict_['path']]
        world.robot.parameters = RobotParameters.parse_obj(dict_['robot']['parameters'])
        world.links = [Link.parse_obj(link) for link in dict_['links']]
    except:
        logging.exception('Could not load from backup')
