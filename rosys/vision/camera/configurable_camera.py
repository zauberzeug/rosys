from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Any, overload

from .camera import Camera


@dataclass(slots=True, kw_only=True)
class ParameterInfo:
    name: str
    min: Any | None = None
    max: Any | None = None
    step: Any | None = None


@dataclass(slots=True, kw_only=True)
class Parameter:
    """A camera parameter which can be adjusted.

    Value ranges can either be a list of options or a min/max value with an optional step size.
    """
    info: ParameterInfo
    value: Any
    setter: Callable
    getter: Callable


class ConfigurableCamera(Camera):
    """A generalized interface for adjusting camera parameters like exposure, brightness or fps."""
    IGNORE_NONE_VALUES = True

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._parameters: dict[str, Parameter] = {}

    @overload
    def _register_parameter(self, name: str, getter: Callable[[], Any], setter: Callable[[Any], None | Awaitable[None]], default_value: Any) -> None:
        ...

    @overload
    def _register_parameter(self, name: str, getter: Callable[[], Any], setter: Callable[[Any], None | Awaitable[None]], default_value: Any,
                            min_value: Any, max_value: Any, step: Any) -> None:
        ...

    def _register_parameter(self, name: str, getter: Callable[[], Any], setter: Callable[[Any], None | Awaitable[None]], default_value: Any,
                            min_value: Any = None, max_value: Any = None, step: Any = None) -> None:
        info = ParameterInfo(name=name, min=min_value, max=max_value, step=step)
        self._parameters[name] = Parameter(info=info, getter=getter, setter=setter, value=default_value)

    async def _apply_parameters(self, new_values: dict[str, Any], force_set: bool = False) -> None:
        if not self.is_connected:
            self._write_values_to_cache(new_values)
            return
        for name, value in new_values.items():
            if name not in self._parameters:
                raise ValueError(f'Cannot set unknown parameter "{name}"')
            if value is None and self.IGNORE_NONE_VALUES:
                continue
            if not force_set and value == self._parameters[name].value:
                continue

            try:
                result = self._parameters[name].setter(value)
                if isinstance(result, Awaitable):
                    await result
            except Exception as e:
                if not self.is_connected:
                    self._write_values_to_cache(new_values)
                    return
                raise e

        await self._update_parameter_cache()

    def _write_values_to_cache(self, new_values: dict[str, Any]) -> None:
        for name, value in new_values.items():
            if name in self._parameters:
                self._parameters[name].value = value

    async def _apply_all_parameters(self) -> None:
        await self._apply_parameters(self.parameters, force_set=True)

    async def _update_parameter_cache(self) -> None:
        if not self.is_connected:
            return
        for param in self._parameters.values():
            try:
                val = param.getter()
                if isinstance(val, Awaitable):
                    val = await val
                if val is None and self.IGNORE_NONE_VALUES:
                    continue
                param.value = val
            except Exception as e:
                if not self.is_connected:
                    return
                raise e

    async def set_parameters(self, new_values: dict[str, Any]) -> None:
        await self._apply_parameters(new_values)

    @property
    def parameters(self) -> dict[str, Any]:
        return {key: param.value for key, param in self._parameters.items()}

    def get_capabilities(self) -> list[ParameterInfo]:
        return [param.info for param in self._parameters.values()]
