from dataclasses import dataclass

from .upload import Upload


@dataclass(slots=True, kw_only=True)
class World:
    upload: Upload = Upload()
