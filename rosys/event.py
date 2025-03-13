from __future__ import annotations

import asyncio
import inspect
import logging
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Any, Generic, ParamSpec

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


P = ParamSpec('P')


class Event(Generic[P]):

    def __init__(self) -> None:
        self.listeners: list[EventListener] = []
        events.append(self)

    def register(self, callback: Callable[P, Any]) -> Event[P]:
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

    def register_ui(self, callback: Callable[P, Any]) -> Event[P]:
        self.register(callback)
        client = context.client
        if not client.shared:
            async def register_disconnect():
                try:
                    await client.connected(timeout=10.0)
                    client.on_disconnect(lambda: self.unregister(callback))
                except TimeoutError:
                    log.warning('could not register disconnect for callback=%s', callback)
                    self.unregister(callback)
            background_tasks.create(register_disconnect())
        return self

    def unregister(self, callback: Callable[P, Any]) -> None:
        self.listeners[:] = [l for l in self.listeners if l.callback != callback]

    async def call(self, *args: P.args, **kwargs: P.kwargs) -> None:
        """Fires event and waits async until all registered listeners are completed"""
        for listener in self.listeners:
            try:
                await invoke(listener.callback, *args, **kwargs)
            except Exception:
                log.exception('could not call listener=%s', listener)

    def emit(self, *args: P.args, **kwargs: P.kwargs) -> None:
        """Fires event without waiting for the result."""
        for listener in self.listeners:
            try:
                result = listener.callback(*args, **kwargs)
                if isinstance(result, Awaitable):
                    if core.loop and core.loop.is_running():
                        name = f'{listener.filepath}:{listener.line}'
                        tasks.append(background_tasks.create(result, name=name))
                    else:
                        startup_coroutines.append(result)
            except Exception:
                log.exception('could not emit listener=%s', listener)

    async def emitted(self, timeout: float | None = None) -> Any:
        """Waits for an event to be emitted and returns its arguments."""
        future: asyncio.Future[Any] = asyncio.Future()

        def callback(*args: P.args, **kwargs: P.kwargs) -> None:  # pylint: disable=unused-argument
            if not future.done():
                future.set_result(args[0] if len(args) == 1 else args if args else None)

        self.register(callback)
        try:
            return await asyncio.wait_for(future, timeout)
        except asyncio.TimeoutError as error:
            raise TimeoutError(f'Timed out waiting for event after {timeout} seconds') from error
        finally:
            self.unregister(callback)

    def __await__(self):
        return self.emitted().__await__()


def reset() -> None:
    for event in events:
        event.listeners.clear()
    for task in tasks:
        task.cancel()
    tasks.clear()
