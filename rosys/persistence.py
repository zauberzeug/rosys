import json
import logging
import os
import sys
from dataclasses import fields
from typing import Any, Protocol, TypeVar

import numpy as np
from dataclasses_json import Exclude, config
from dataclasses_json.core import _asdict, _decode_dataclass

exclude = config(exclude=Exclude.ALWAYS)


def to_dict(obj: Any) -> dict[str, Any]:
    return _asdict(obj, False)


def from_dict(cls: type, d: dict[str, Any]) -> Any:
    return _decode_dataclass(cls, d, False)


T = TypeVar('T')


def replace_dict(old_dict: dict[str, T], cls: type, new_dict: dict[str, T]) -> None:
    '''Replace content of `old_dict` with keys and values from `new_dict`.'''
    old_dict.clear()
    old_dict.update({key: from_dict(cls, value) for key, value in new_dict.items()})


def replace_list(old_list: list[T], cls: type, new_list: list[T]) -> None:
    '''Replace content of `old_list` with items from `new_list`.'''
    old_list.clear()
    old_list.extend(from_dict(cls, value) for value in new_list)


def replace_dataclass(old_dataclass: Any, new_dict: dict[str, Any]) -> None:
    '''Replace content of `old_dataclass` with content from `new_dict`.'''
    for field in fields(old_dataclass):
        setattr(old_dataclass, field.name, new_dict.get(field.name))


backup_path = os.path.expanduser('~/.rosys')
log = logging.getLogger('rosys.persistence')

is_test = 'pytest' in sys.modules


class PersistentActor(Protocol):
    needs_backup: bool

    def backup(self, data: dict[str, Any]) -> None:
        ...

    def restore(self) -> dict[str, Any]:
        ...


actors: list[PersistentActor] = []


def register(actor: PersistentActor) -> None:
    if not is_test:
        actors.append(actor)


class Encoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.floating):
            return float(obj)
        return json.JSONEncoder.default(self, obj)


def backup(force: bool = False) -> None:
    for actor in actors:
        if not actor.needs_backup and not force:
            continue
        if not os.path.exists(backup_path):
            os.makedirs(backup_path)
        filepath = f'{backup_path}/{actor.__module__}.json'
        with open(filepath, 'w') as f:
            json.dump(actor.backup(), f, indent=4, cls=Encoder)
        actor.needs_backup = False


def restore() -> None:
    for actor in actors:
        filepath = f'{backup_path}/{actor.__module__}.json'
        if not os.path.exists(filepath):
            log.warning(f'Backup file "{filepath}" not found.')
            continue
        with open(filepath) as f:
            actor.restore(json.load(f))
