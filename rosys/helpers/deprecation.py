import warnings
from collections.abc import Callable
from functools import wraps


def deprecated_param(param_name: str) -> Callable:
    """Mark a function parameter as deprecated."""
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            if param_name in kwargs:
                warnings.warn(
                    f'The parameter "{param_name}" is deprecated and will be removed in future versions.',
                    category=DeprecationWarning,
                    stacklevel=2,
                )
            return func(*args, **kwargs)
        return wrapper
    return decorator
