import pylab as pl
import numpy as np


def plot_robot(robot_renderer, pose, color='C0', **kwargs):
    poly = pl.matplotlib.patches.Polygon(robot_renderer.outline, color='none', ec=color, joinstyle='round', **kwargs)
    triangle = pl.matplotlib.patches.Polygon(
        [[0, 0], [-0.1, -0.05], [-0.1, 0.05]], color='none', ec=color, joinstyle='round', **kwargs)
    pl.gca().add_patch(poly)
    pl.gca().add_patch(triangle)
    move([poly, triangle], pose)


def move(patches, pose):
    r = pl.matplotlib.transforms.Affine2D().rotate_deg(np.rad2deg(pose[2]))
    t = pl.matplotlib.transforms.Affine2D().translate(pose[0], pose[1])
    for patch in patches:
        patch.set_transform(r + t + pl.gca().transData)


def plot_spline(spline, *args, backward=False, **kwargs):
    t = np.linspace(0, 1, 20)
    return pl.plot(spline.x(t), spline.y(t), *args, alpha=0.5 if backward else 1.0, **kwargs)


def plot_path(path, *args, **kwargs):
    return [
        plot_spline(step.spline, *args, backward=step.backward, **kwargs)
        for step in path
    ]


def show_obstacle_map(obstacle_map):
    cmap = pl.matplotlib.colors.ListedColormap([[0, 0, 0, 0.1], 'C3'])
    extent = bbox_to_extent(obstacle_map.grid.bbox)
    pl.imshow(obstacle_map.map, cmap=cmap, interpolation='nearest', extent=extent, alpha=0.5)
    plot_bbox(obstacle_map.grid.bbox, 'C0--', lw=0.2, dashes=[20, 20])


def show_distance_map(distance_map):
    extent = bbox_to_extent(distance_map.grid.bbox)
    pl.imshow(distance_map.map, cmap=pl.cm.gray, interpolation='nearest', extent=extent)
    plot_bbox(distance_map.grid.bbox, 'C0--', lw=0.2, dashes=[20, 20])


def bbox_to_extent(bbox):
    return [bbox[0], bbox[0] + bbox[2], bbox[1] + bbox[3], bbox[1]]


def plot_bbox(bbox, *args, **kwargs):
    pl.plot([bbox[0], bbox[0] + bbox[2], bbox[0] + bbox[2], bbox[0], bbox[0]],
            [bbox[1], bbox[1], bbox[1] + bbox[3], bbox[1] + bbox[3], bbox[1]],
            *args, **kwargs)
