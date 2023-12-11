from __future__ import annotations

import asyncio
import inspect
import logging
from dataclasses import dataclass
from typing import Awaitable, Callable

from nicegui import background_tasks, context, core

from .helpers import invoke

startup_coroutines: list[Awaitable] = []
tasks: list[asyncio.Task] = []
log = logging.getLogger('rosys.event')
events: list[Event] = []


@dataclass(slots=True, kw_only=True)
class EventListener:
    callback: Callable
    filepath: str
    line: int


class Event:

    def __init__(self) -> None:
        self.listeners: list[EventListener] = []
        events.append(self)

    def register(self, callback: Callable) -> Event:
        if not callable(callback):
            raise ValueError('non-callable callback')
        if any(l.callback == callback for l in self.listeners):
            return self  # NOTE: don't add duplicate listeners
        frame = inspect.currentframe()
        assert frame is not None
        frame = frame.f_back
        assert frame is not None
        self.listeners.append(EventListener(callback=callback, filepath=frame.f_code.co_filename, line=frame.f_lineno))
        return self

    def register_ui(self, callback: Callable) -> Event:
        self.register(callback)
        client = context.get_client()
        if not client.shared:
            async def register_disconnect():
                try:
                    await client.connected(timeout=10.0)
                    client.on_disconnect(lambda: self.unregister(callback))
                except TimeoutError:
                    log.warning(f'could not register disconnect for {callback=}')
                    self.unregister(callback)
            background_tasks.create(register_disconnect())
        return self

    def unregister(self, callback: Callable) -> None:
        self.listeners[:] = [l for l in self.listeners if l.callback != callback]

    async def call(self, *args) -> None:
        """Fires event and waits async until all registered listeners are completed"""
        for listener in self.listeners:
            try:
                await invoke(listener.callback, *args)
            except Exception:
                log.exception(f'could not call {listener=}')

    def emit(self, *args) -> None:
        """Fires event without waiting for the result."""
        for listener in self.listeners:
            try:
                result = invoke(listener.callback, *args)
                if isinstance(result, Awaitable):
                    if core.loop and core.loop.is_running():
                        name = f'{listener.filepath}:{listener.line}'
                        tasks.append(background_tasks.create(result, name=name))
                    else:
                        startup_coroutines.append(result)
            except Exception:
                log.exception(f'could not emit {listener=}')


def reset() -> None:
    for event in events:
        event.listeners.clear()
    for task in tasks:
        task.cancel()
    tasks.clear()
