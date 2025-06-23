import asyncio
from dataclasses import dataclass, field
from typing import Any

import pytest

from rosys import persistence
from rosys.testing import forward


@dataclass
class A:
    s: str
    b: bool
    e: int = field(default=0, metadata=persistence.exclude)


@dataclass
class Model:
    x: int
    y: float
    a: A
    l: list[A]
    d: dict[str, A]
    e: int = field(default=0, metadata=persistence.exclude)


def test_conversion_to_and_from_dict() -> None:
    model = Model(
        x=42,
        y=3.14,
        a=A('foo', True, 1),
        l=[A('foo', True, 2), A('bar', True, 3)],
        d={'a': A('foo', True, 4), 'b': A('bar', True, 5)},
        e=101,
    )
    d = persistence.to_dict(model)
    assert d == {
        'x': 42,
        'y': 3.14,
        'a': {'s': 'foo', 'b': True},
        'l': [{'s': 'foo', 'b': True}, {'s': 'bar', 'b': True}],
        'd': {'a': {'s': 'foo', 'b': True}, 'b': {'s': 'bar', 'b': True}},
    }
    model_ = persistence.from_dict(Model, d)
    model.e = 0
    model.a.e = 0
    model.l[0].e = 0
    model.l[1].e = 0
    model.d['a'].e = 0
    model.d['b'].e = 0
    assert model_ == model, 'the model should equal the original model (up to excluded fields)'


def test_dict_replacement() -> None:
    target = {
        'a': A('foo', True),
        'b': A('bar', True),
    }
    target_id = id(target)
    persistence.replace_dict(target, A, {'c': {'s': 'buz', 'b': False}})
    assert id(target) == target_id, 'target should still be the same object'
    assert target == {'c': A('buz', False)}, 'target content should be completely replaced with new content'


@pytest.mark.usefixtures('rosys_integration')
async def test_persistable() -> None:
    class MyPersistable(persistence.Persistable):
        def __init__(self, x: int, y: float) -> None:
            super().__init__()
            self.x = x
            self.y = y

        def backup_to_dict(self) -> dict[str, Any]:
            return {'x': self.x, 'y': self.y}

        def restore_from_dict(self, data: dict[str, Any]) -> None:
            self.x = data['x']
            self.y = data['y']

    persistable = MyPersistable(x=42, y=3.14)
    persistable.persistent(restore=False, allow_tests=True)
    persistable.sync_backup()

    # pylint: disable=protected-access
    assert persistable._filepath is not None
    assert persistable._filepath.read_text() == '{"x": 42, "y": 3.14}'

    with pytest.raises(RuntimeError, match='already persistent'):
        persistable.persistent(allow_tests=True)

    persistable.x = 100
    persistable.sync_restore()
    assert persistable.x == 42
    assert persistable.y == 3.14

    persistable.x = 100
    persistable.request_backup()
    await forward(seconds=15)
    await asyncio.sleep(1.0)  # give some time for IO
    assert persistable._filepath.read_text() == '{"x": 100, "y": 3.14}'
