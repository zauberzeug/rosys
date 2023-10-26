import abc
from dataclasses import dataclass
from typing import Any, Callable, Optional, overload


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
    value: Optional[Any] = None
    setter: Callable
    getter: Callable


class ConfigurableCameraMixin(abc.ABC):
    """A generalized interface for adjusting camera parameters like exposure, brightness or fps."""
    _parameters: dict[str, Any]

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._parameters = {}

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
                await self._parameters[param].setter(new_values[param])

    async def _apply_all_parameters(self) -> None:
        await self._apply_parameters({param: self._parameters[param].value for param in self._parameters})

    async def _update_parameter_values(self) -> None:
        for param in self._parameters.values():
            val = await param.getter()
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
