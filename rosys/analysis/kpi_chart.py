from dataclasses import dataclass


@dataclass
class KpiChart:
    title: str
    indicators: dict[str, str]
    colormap: str = ''  # choose from matplotlib colormaps
    scale: float = 1.0
    unit: str = ''
