#!/usr/bin/env python3
from typing import Any

from nicegui import app, ui

from rosys import persistence


class Model(persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()
        self.value: float = 5.0

    def set_value(self, value: float) -> None:
        self.value = value
        self.request_backup()

    def restore(self, data: dict[str, Any]) -> None:
        self.value = data.get('value', 1.0)

    def backup(self) -> dict[str, Any]:
        return {
            'value': self.value,
        }


def start():
    model = Model()
    model.set_value(3.0)
    ui.slider(min=0, max=10.0, step=0.1, on_change=model.request_backup).bind_value(
        model, 'value').props('label-always')


app.on_startup(start)

ui.run(title='RoSys')
