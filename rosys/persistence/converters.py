import logging
from dataclasses import fields
from typing import Any, TypeVar

from dataclasses_json.core import _asdict, _decode_dataclass

log = logging.getLogger('rosys.persistence.converters')

T = TypeVar('T')


def to_dict(obj: Any) -> dict[str, Any]:
    """Convert an object `obj` to a serializable dict."""
    return _asdict(obj, False)


def from_dict(cls: type[T], d: dict[str, Any]) -> T:
    """Convert a serializable dict `d` to object of type `cls`."""
    try:
        return _decode_dataclass(cls, d, False)
    except Exception:
        log.exception('Failed to decode %s from %s', cls, d)
        raise


def replace_dict(old_dict: dict[str, Any], cls: type, new_dict: dict[str, Any]) -> None:
    """Replace content of `old_dict` with keys and values from `new_dict`."""
    old_dict.clear()
    old_dict.update({key: from_dict(cls, value) for key, value in new_dict.items()})


def replace_list(old_list: list[Any], cls: type, new_list: list[Any]) -> None:
    """Replace content of `old_list` with items from `new_list`."""
    old_list.clear()
    old_list.extend(from_dict(cls, value) for value in new_list)


def replace_set(old_set: set[Any], cls: type, new_set: set[Any]) -> None:
    """Replace content of `old_set` with items from `new_set`."""
    old_set.clear()
    old_set.update(from_dict(cls, value) for value in new_set)


def replace_dataclass(old_dataclass: Any, new_dict: dict[str, Any]) -> None:
    """Replace content of `old_dataclass` with content from `new_dict`."""
    for field in fields(old_dataclass):
        setattr(old_dataclass, field.name, new_dict.get(field.name))
