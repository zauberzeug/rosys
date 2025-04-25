from collections.abc import Callable

# This module gives access to some RoSys functions without needing to import rosys itself.
# This is a workaround to avoid a circular dependency for modules that are part of rosys (e.g. config.py)
# and would need to import rosys (via persistable.py).

on_repeat: Callable[[Callable, float], None]
on_startup: Callable[[Callable], None]
on_shutdown: Callable[[Callable], None]
