from dataclasses import dataclass

import pytest

from rosys.testing import approx


def test_approx_bools():
    approx(True, True)
    with pytest.raises(AssertionError):
        approx(True, False)


def test_approx_numbers():
    approx(1, 1)
    approx(2, 2.0)
    approx(3.0, 3.0)
    with pytest.raises(AssertionError):
        approx(1, 2)
        approx(1, 2.0)
        approx(1.0, 2)


def test_approx_strings():
    approx('abc', 'abc')
    with pytest.raises(AssertionError):
        approx('abc', 'abd')


def test_approx_lists():
    approx([True, 1, 'a', [1.1, 1.2]], [True, 1, 'a', [1.1, 1.2]])
    with pytest.raises(AssertionError):
        approx([True, 1, 'a'], [True, 1, 'b'])


def test_approx_dicts():
    approx({'a': 1, 'b': 2}, {'b': 2, 'a': 1})
    with pytest.raises(AssertionError):
        approx({'a': 1, 'b': 2}, {'a': 1, 'b': 3})
        approx({'a': 1, 'b': 2}, {'a': 1})


def test_approx_objects():
    class A:
        def __init__(self, x: float) -> None:
            self.x = x
    approx(A(1), A(1))
    with pytest.raises(AssertionError):
        approx(A(1), A(2))


def test_approx_dataclasses():
    @dataclass
    class A:
        x: float
    approx(A(1), A(1))
    with pytest.raises(AssertionError):
        approx(A(1), A(2))


def test_approx_dataclasses_with_kw_and_slots():
    @dataclass(slots=True, kw_only=True)
    class A:
        x: float
    approx(A(x=1), A(x=1))
    with pytest.raises(AssertionError):
        approx(A(x=1), A(x=2))
