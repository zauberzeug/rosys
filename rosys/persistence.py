import logging
import json
import os
from .world.world import World
from .world.camera import Camera
from .world.robot import RobotParameters
from .world.spline import Spline


def backup(world: World):

    exclude = {'projection', 'synchronization'}
    dict_ = {
        'cameras': {mac: camera.dict(exclude=exclude) for mac, camera in world.cameras.items()},
        'path': [spline.dict() for spline in world.path],
        'robot': {'parameters': world.robot.parameters.dict()}
    }
    os.makedirs('/data/backup', exist_ok=True)
    with open('/data/backup/world.json', 'w') as f:
        json.dump(dict_, f)


def restore(world: World):

    try:
        with open('/data/backup/world.json', 'r') as f:
            dict_ = json.load(f)
        world.cameras = {mac: Camera.parse_obj(camera) for mac, camera in dict_['cameras'].items()}
        world.path = [Spline.parse_obj(spline) for spline in dict_['path']]
        world.robot.parameters = RobotParameters.parse_obj(dict_['robot']['parameters'])
    except:
        logging.exception('could not load from backup')
