from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class BmsState:
    percentage: float | None = None
    voltage: float | None = None
    current: float | None = None
    temperature: float | None = None
    is_charging: bool | None = None
    last_update: float = 0

    @property
    def short_string(self) -> str:
        parts = []
        parts += [f'{self.percentage:.1f}%'] if self.percentage is not None else []
        parts += [f'{self.voltage:.1f}V'] if self.voltage is not None else []
        parts += [f'{self.current:.1f}A'] if self.current is not None else []
        parts += [f'{self.temperature:.1f}°C'] if self.temperature is not None else []
        return ', '.join(parts) or '?'
