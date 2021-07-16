from .world.world import World
from .world.camera import Camera
import logging
import json
import os


def backup(world: World):

    json = world.json(include={'cameras': ...})
    os.makedirs('/data/backup', exist_ok=True)
    with open('/data/backup/world.json', 'w') as f:
        f.write(json)


def restore(world: World):
    try:
        with open('/data/backup/world.json', 'r') as f:
            cameras = json.load(f)['cameras']
        world.cameras = {k: Camera.parse_obj(v) for k, v in cameras.items()}
    except:
        logging.exception('could not load from backup')
