import os
import svgpathtools
import numpy as np
from rosys.world.robot import Pose
from rosys.world.point import Point
from rosys.world.spline import Spline

filepath = '/data/drawings/default.svg'
path = os.path.dirname(filepath)
if not os.path.exists(path):
    os.makedirs(path)


def store(content: bytearray):

    with open(filepath, 'wb') as f:
        f.write(content)


def load(pose: Pose, target_size: float = 2.0) -> list[Spline]:

    if not os.path.exists(filepath):
        return []
    paths = svgpathtools.svg2paths(filepath)[0]
    x_min, x_max, y_min, y_max = svgpathtools.paths2svg.big_bounding_box(paths)
    center = Point(x=(x_min + x_max) / 2, y=(y_min + y_max) / 2)
    scale = target_size / max(x_max - x_min, y_max - y_min)
    splines = []
    for path in paths:
        path = path.translated(-center.to_complex())
        path = path.scaled(scale, -scale)
        path = path.rotated(np.rad2deg(pose.yaw), origin=0)
        path = path.translated(pose.point.to_complex())
        for line in path:
            if type(line) == svgpathtools.path.Line:
                start = Point.from_complex(line.start)
                end = Point.from_complex(line.end)
                splines.append(Spline(
                    start=start,
                    control1=start.interpolate(end, 1 / 3),
                    control2=start.interpolate(end, 2 / 3),
                    end=end,
                ))
            if type(line) == svgpathtools.path.CubicBezier:
                splines.append(Spline(
                    start=Point.from_complex(line.start),
                    control1=Point.from_complex(line.control1),
                    control2=Point.from_complex(line.control2),
                    end=Point.from_complex(line.end),
                ))
    return splines
