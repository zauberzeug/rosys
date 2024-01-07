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


class ConfigurableCamera(Camera):
    """A generalized interface for adjusting camera parameters like exposure, brightness or fps."""
    IGNORE_NONE_VALUES = True

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._parameters: dict[str, Parameter] = {}
        self._pending_operations: int = 0

    @overload
    def _register_parameter(self, name: str, getter: Callable[[], Any], setter: Callable[[Any], None], default_value: Any) -> None:
        ...

    @overload
    def _register_parameter(self, name: str, getter: Callable[[], Any], setter: Callable[[Any], None], default_value: Any,
                            min_value: Any, max_value: Any, step: Any) -> None:
        ...

    def _register_parameter(self, name: str, getter: Callable[[], Any], setter: Callable[[Any], None], default_value: Any,
                            min_value: Any = None, max_value: Any = None, step: Any = None) -> None:
        info = ParameterInfo(name=name, min=min_value, max=max_value, step=step)
        self._parameters[name] = Parameter(info=info, getter=getter, setter=setter, value=default_value)

    def _apply_parameters(self, new_values: dict[str, Any]) -> None:
        if not self.is_connected:
            return
        for name, value in new_values.items():
            if name not in self._parameters:
                raise ValueError(f'Cannot set unknown parameter "{name}"')
            if value is None and self.IGNORE_NONE_VALUES:
                continue
            if value == self._parameters[name].value:
                continue
            self._parameters[name].setter(value)
        self._update_parameter_cache()

    def _apply_all_parameters(self) -> None:
        self._apply_parameters(self.parameters)

    def _update_parameter_cache(self) -> None:
        if not self.is_connected:
            return
        for param in self._parameters.values():
            val = param.getter()
            if val is None and self.IGNORE_NONE_VALUES:
                continue
            param.value = val

    def set_parameters(self, new_values: dict[str, Any]) -> None:
        self._apply_parameters(new_values)

    @property
    def parameters(self) -> dict[str, Any]:
        return {key: param.value for key, param in self._parameters.items()}

    def get_capabilities(self) -> list[ParameterInfo]:
        return [param.info for param in self._parameters.values()]
