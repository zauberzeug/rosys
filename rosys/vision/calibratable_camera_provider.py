from typing import Protocol, TypeVar

from .camera import CalibratableCamera

T = TypeVar('T', bound=CalibratableCamera)


class CalibratableCameraProvider(Protocol[T]):

    @property
    def cameras(self) -> dict[str, T]:
        ...
