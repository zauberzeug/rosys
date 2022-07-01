import json
import logging
import os.path
from typing import Any, Protocol

backup_path = os.path.expanduser('~/.rosys')
log = logging.getLogger('rosys.persistence')


class PersistentActor(Protocol):
    needs_backup: bool

    def backup(self, data: dict[str, Any]) -> None:
        ...

    def restore(self) -> dict[str, Any]:
        ...


actors: list[PersistentActor] = []


def register(actor: PersistentActor):
    actors.append(actor)


def backup() -> None:
    for actor in actors:
        if not actor.needs_backup:
            continue
        filepath = f'{backup_path}/{actor.__module__}.json'
        with open(filepath, 'w') as f:
            json.dump(actor.backup(), f)
        actor.needs_backup = False


def restore() -> None:
    for actor in actors:
        filepath = f'{backup_path}/{actor.__module__}.json'
        if not os.path.exists(filepath):
            log.warning(f'Backup file "{filepath}" not found.')
            continue
        with open(filepath) as f:
            actor.restore(json.load(f))
