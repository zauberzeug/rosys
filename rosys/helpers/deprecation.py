import warnings
from collections.abc import Callable
from functools import wraps


def deprecated_param(param_name: str, *, remove_in_version: str | None = None, stacklevel: int = 2) -> Callable:
    """Mark a function parameter as deprecated."""
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            if param_name in kwargs:
                warnings.warn(
                    f'The parameter "{param_name}" is deprecated and will be removed in '
                    f'{("RoSys " + remove_in_version) if remove_in_version else "a future version"}.',
                    category=DeprecationWarning,
                    stacklevel=stacklevel,
                )
            return func(*args, **kwargs)
        return wrapper
    return decorator


def deprecated_function(*, remove_in_version: str | None = None, stacklevel: int = 2) -> Callable:
    """Mark a function as deprecated."""
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            warnings.warn(
                f'The function "{func.__name__}" is deprecated and will be removed in '
                f'{("RoSys " + remove_in_version) if remove_in_version else "a future version"}.',
                category=DeprecationWarning,
                stacklevel=stacklevel,
            )
            return func(*args, **kwargs)
        return wrapper
    return decorator


def deprecated_class(*, remove_in_version: str | None = None, stacklevel: int = 2) -> Callable:
    """Mark a class as deprecated."""
    def decorator(cls: type) -> type:
        original_init = cls.__init__

        def new_init(self, *args, **kwargs):
            warnings.warn(
                f'The class "{cls.__name__}" is deprecated and will be removed in '
                f'{("RoSys " + remove_in_version) if remove_in_version else "a future version"}.',
                category=DeprecationWarning,
                stacklevel=stacklevel,
            )
            original_init(self, *args, **kwargs)

        cls.__init__ = new_init
        return cls
    return decorator
