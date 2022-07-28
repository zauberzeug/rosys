from dataclasses import dataclass


@dataclass(slots=True, kw_only=True, frozen=True)
class Config:
    ui_update_interval: float = 0.1
