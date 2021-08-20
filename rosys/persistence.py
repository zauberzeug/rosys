import logging
import json
import os
from .world.world import World
from .world.camera import Camera
from .world.robot import RobotParameters
from .world.spline import Spline
from .world.link import Link

filepath = '/data/backup/world.json'


def dump(world: World) -> dict:

    exclude = {'projection', 'synchronization'}
    return {
        'cameras': {mac: camera.dict(exclude=exclude) for mac, camera in world.cameras.items()},
        'path': [spline.dict() for spline in world.path],
        'robot': {'parameters': world.robot.parameters.dict()},
        'links': [link.dict() for link in world.links],
    }


def load(world: World, dict: dict):

    world.cameras = {mac: Camera.parse_obj(camera) for mac, camera in dict['cameras'].items()}
    world.path = [Spline.parse_obj(spline) for spline in dict['path']]
    world.robot.parameters = RobotParameters.parse_obj(dict['robot']['parameters'])
    world.links = [Link.parse_obj(link) for link in dict['links']]


def backup(world: World):

    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'w') as f:
        json.dump(dump(world), f)


def restore(world: World):

    if not os.path.exists(filepath):
        logging.warning('No backup file found.')
        return

    try:
        with open(filepath, 'r') as f:
            load(world, json.load(f))
    except:
        logging.exception('Could not load from backup')
