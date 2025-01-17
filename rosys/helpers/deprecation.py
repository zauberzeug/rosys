import warnings
from functools import wraps


def deprecated_param(param_name):
    """Usage:
    @deprecated_param('my_param')
    def my_function(param1, my_param=None):"""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if param_name in kwargs:
                warnings.warn(
                    f"The '{param_name}' parameter is deprecated and will be removed in future versions.",
                    stacklevel=2,
                )
            return func(*args, **kwargs)
        return wrapper
    return decorator
