import os
from typing import List
import svgpathtools
from rosys.world.point import Point
from rosys.world.spline import Spline

filepath = '/data/drawings/default.svg'
path = os.path.dirname(filepath)
if not os.path.exists(path):
    os.makedirs(path)


def store(content: bytearray):

    with open(filepath, 'wb') as f:
        f.write(content)


def load() -> List[Spline]:

    paths, _ = svgpathtools.svg2paths(filepath)
    for path in paths:
        for line in path:
            if type(line) == svgpathtools.path.Line:
                start = Point.from_complex(line.start)
                end = Point.from_complex(line.end)
                yield Spline(
                    start=start,
                    control1=start.interpolate(end, 1/3),
                    control2=start.interpolate(end, 2/3),
                    end=end,
                )
            if type(line) == svgpathtools.path.CubicBezier:
                yield Spline(
                    start=Point.from_complex(line.start),
                    control1=Point.from_complex(line.control1),
                    control2=Point.from_complex(line.control2),
                    end=Point.from_complex(line.end),
                )
