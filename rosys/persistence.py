import logging
import json
import os
from .world.world import World
from .world.camera import Camera


def backup(world: World):

    exclude = {'projection', 'synchronization'}
    dict_ = {mac: camera.dict(exclude=exclude) for mac, camera in world.cameras.items()}
    os.makedirs('/data/backup', exist_ok=True)
    with open('/data/backup/world.json', 'w') as f:
        json.dump(dict_, f)


def restore(world: World):
    try:
        with open('/data/backup/world.json', 'r') as f:
            dict_ = json.load(f)
        world.cameras = {mac: Camera.parse_obj(camera) for mac, camera in dict_.items()}
    except:
        logging.exception('could not load from backup')
