from dataclasses import dataclass, field

from rosys import persistence


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
        'l': [1, 2, 3],
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
