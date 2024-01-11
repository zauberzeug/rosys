from dataclasses import dataclass


@dataclass
class KpiChart:
    title: str
    indicators: dict[str, str]
    color: str = ""  # choose from matplotlib colormaps
