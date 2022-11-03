#!/usr/bin/env python3
from typing import Any

from nicegui import ui
from rosys import persistence


class Model:

    def __init__(self) -> None:
        self.value: float = 1.0

        persistence.register(self)
        self.needs_backup = False

    def restore(self, data: dict[str, Any]) -> None:
        self.value = data.get('value', 1.0)

    def backup(self) -> dict[str, Any]:
        return {
            'value': self.value,
        }


model = Model()
ui.slider(min=0, max=10.0, step=0.1).bind_value(model, 'value').props('label-always')

ui.run(title='RoSys')
