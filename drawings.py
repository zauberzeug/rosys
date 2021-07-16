import os
import svgpathtools
from svgpathtools.paths2svg import big_bounding_box
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
    xmin, xmax, ymin, ymax = big_bounding_box(paths)
    xlen = xmax-xmin
    ylen = ymax-ymin
    center = Point(x=xlen/2, y=ylen/2)
    splines = []
    scale = target_size/max(xlen, ylen)
    ic(center, scale, center*scale)
    for path in paths:
        path = path.translated((center*-1).to_complex())
        path = path.scaled(scale, scale, origin=0)  # , origin=
        path = path.rotated(np.rad2deg(pose.yaw), origin=0)
        path = path.translated(Point(x=pose.x+1, y=pose.y-0.5).to_complex())
        for line in path:
            if type(line) == svgpathtools.path.Line:
                start = Point.from_complex(line.start)
                end = Point.from_complex(line.end)
                splines.append(Spline(
                    start=start,
                    control1=start.interpolate(end, 1/3),
                    control2=start.interpolate(end, 2/3),
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


def scale(path: list[Spline], target_size: float = 1.0) -> list[Spline]:

    min_x = np.inf
    min_y = np.inf
    max_x = -np.inf
    max_y = -np.inf
    for spline in path:
        turning_points = spline.turning_points()
        turning_x = [spline.x(t) for t in turning_points]
        turning_y = [spline.y(t) for t in turning_points]
        min_x = min([min_x, spline.start.x, spline.end.x] + turning_x)
        min_y = min([min_y, spline.start.y, spline.end.y] + turning_y)
        max_x = max([max_x, spline.start.x, spline.end.x] + turning_x)
        max_y = max([max_y, spline.start.y, spline.end.y] + turning_y)
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    range_x = max_x - min_x
    range_y = max_y - min_y
    range = (range_x + range_y) / 2
    factor = target_size / range
    return [
        Spline(
            start=Point(x=(spline.start.x - center_x) * factor,
                        y=(spline.start.y - center_y) * factor),
            control1=Point(x=(spline.control1.x - center_x) * factor,
                           y=(spline.control1.y - center_y) * factor),
            control2=Point(x=(spline.control2.x - center_x) * factor,
                           y=(spline.control2.y - center_y) * factor),
            end=Point(x=(spline.end.x - center_x) * factor,
                      y=(spline.end.y - center_y) * factor),
        )
        for spline in path
    ]
