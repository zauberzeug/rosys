from dataclasses import dataclass
from typing import Any, Callable, Optional, overload

from .camera import Camera


@dataclass(slots=True, kw_only=True)
class ParameterInfo:
    name: str
    min: Optional[Any] = None
    max: Optional[Any] = None
    step: Optional[Any] = None


@dataclass(slots=True, kw_only=True)
class Parameter:
    """A camera parameter which can be adjusted.

    Value ranges can either be a list of options or a min/max value with an optional step size.
    """
    info: ParameterInfo
    value: Any
    setter: Callable
    getter: Callable


class ConfigurableCameraMixin(Camera):
    """A generalized interface for adjusting camera parameters like exposure, brightness or fps."""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._parameters: dict[str, Parameter] = {}
        self._pending_operations: int = 0

    @overload
    def _register_parameter(self, name: str, getter: Callable, setter: Callable, default_value: Any) -> None:
        ...

    @overload
    def _register_parameter(self, name: str, getter: Callable, setter: Callable, default_value: Any,
                            min_value: Any, max_value: Any, step: Any) -> None:
        ...

    def _register_parameter(self, name: str, getter: Callable, setter: Callable, default_value: Any,
                            min_value: Any = None, max_value: Any = None, step: Any = None) -> None:
        info = ParameterInfo(name=name, min=min_value, max=max_value, step=step)
        self._parameters[name] = Parameter(info=info, getter=getter, setter=setter, value=default_value)

    async def _apply_parameters(self, new_values: dict[str, Any]) -> None:
        for param in new_values:
            if param not in self._parameters:
                raise ValueError(f'Cannot set unknown parameter "{param}"')
            if new_values[param] is not None and new_values[param] != self._parameters[param].value:
                await self._secure_parameter_setter(self._parameters[param].setter, new_values[param])

    async def _apply_all_parameters(self) -> None:
        await self._apply_parameters({param: self._parameters[param].value for param in self._parameters})

    async def _update_parameter_values(self) -> None:
        for param in self._parameters.values():
            val = await self._secure_parameter_getter(param.getter)
            if val is not None:
                param.value = val

    async def set_parameters(self, new_values: dict[str, Any]) -> None:
        await self._apply_parameters(new_values)
        await self._update_parameter_values()

    @property
    def parameters(self) -> dict[str, Any]:
        return {param: self._parameters[param].value for param in self._parameters}

    def get_capabilities(self) -> list[ParameterInfo]:
        return [param.info for param in self._parameters.values()]

    async def _secure_parameter_getter(self, getter) -> Any:
        async with self._device_connection():
            if not self.is_connected:
                return None
            self._pending_operations += 1

        value = await getter()

        async with self.device_connection_lock:
            self._pending_operations -= 1
            self.device_connection_lock.notify_all()

        return value

    async def _secure_parameter_setter(self, setter, value) -> None:
        async with self._device_connection():
            if not self.is_connected:
                return None
            self._pending_operations += 1

        await setter(value)

        async with self._device_connection():
            self._pending_operations -= 1
            self.device_connection_lock.notify_all()
