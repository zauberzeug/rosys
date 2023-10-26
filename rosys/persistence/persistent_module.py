import abc
from typing import Any

from . import registry


class PersistentModule(abc.ABC):
    USE_PERSISTENCE: bool = True

    def __init__(self, *, persistence_key: str | None = None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.needs_backup: bool = False
        if self.USE_PERSISTENCE:
            registry.register(self, persistence_key)

    @abc.abstractmethod
    def backup(self) -> dict[str, Any]:
        """Return a JSON-serializable dict with all data that should be backed up."""

    @abc.abstractmethod
    def restore(self, data: dict[str, Any]) -> None:
        """Restore data from a JSON-serializable dict."""

    def request_backup(self) -> None:
        """Mark this module as changed and in need of a backup."""
        self.needs_backup = True
