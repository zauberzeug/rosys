from typing import Protocol

from nicegui import Event


class VelocityProvider(Protocol):
    VELOCITY_MEASURED: Event
