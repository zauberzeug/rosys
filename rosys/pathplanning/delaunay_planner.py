import itertools
import logging
from dataclasses import dataclass
from functools import lru_cache

import networkx as nx
import numpy as np
from scipy import ndimage, spatial

from ..driving import PathSegment
from ..geometry import Point, Pose, PoseStep, Spline
from ..helpers import angle
from .area import Area
from .delaunay_pose_group import DelaunayPoseGroup
from .fast_spline import FastSpline
from .grid import Grid
from .obstacle import Obstacle
from .obstacle_map import ObstacleMap

GRID_RESOLUTION = 1.0
MIN_MARGIN = 1.0

TRY_SINGLE_PATH = True
"""Try to find a collision-free simple path between start and goal.

See https://trello.com/c/FtP4yHqA/777#comment-62d0323e97ba19392bcbceb8 for more information.
"""

TRY_SINGLE_SHUNTING = True
"""Try to find collision-free path with minimal switching (single shunting)."""


class DelaunayPlanner:

    def __init__(self, robot_outline: list[tuple[float, float]]) -> None:
        self.robot_outline = robot_outline
        self.areas: list[Area] = []
        self.obstacles: list[Obstacle] = []
        self.obstacle_map: ObstacleMap | None = None
        self.tri_points: np.ndarray | None = None
        self.tri_mesh: spatial.Delaunay | None = None
        self.pose_groups: list[DelaunayPoseGroup] | None = None
        self.graph: nx.DiGraph | None = None
        self.log = logging.getLogger('rosys.delaunay_planner')

    def update_map(self, areas: list[Area], obstacles: list[Obstacle], additional_points: list[Point],
                   deadline: float) -> None:
        if self.obstacle_map and \
                self.areas == areas and \
                self.obstacles == obstacles and \
                all(self.obstacle_map.grid.contains(point, padding=1.0) for point in additional_points):
            return
        self.areas = areas
        self.obstacles = obstacles
        self._create_obstacle_map(additional_points, deadline)
        self._create_graph()

    def grow_map(self, points: list[Point], deadline: float) -> None:
        if self.obstacle_map is not None and \
                all(self.obstacle_map.grid.contains(point, padding=1.0) for point in points):
            return
        if self.obstacle_map is not None:
            bbox = self.obstacle_map.grid.bbox
            points.append(Point(x=bbox[0],         y=bbox[1]))
            points.append(Point(x=bbox[0]+bbox[2], y=bbox[1]))
            points.append(Point(x=bbox[0],         y=bbox[1]+bbox[3]))
            points.append(Point(x=bbox[0]+bbox[2], y=bbox[1]+bbox[3]))
        self._create_obstacle_map(points, deadline)
        self._create_graph()

    def _create_obstacle_map(self, additional_points: list[Point], deadline: float) -> None:
        points = [p for obstacle in self.obstacles for p in obstacle.outline]
        points += [p for area in self.areas for p in area.outline]
        points += additional_points
        grid = Grid.from_points(points, pixel_size=0.1, num_layers=36, padding=1.0)
        self.obstacle_map = ObstacleMap.from_world(self.robot_outline, self.areas, self.obstacles, grid, deadline)

    def _create_graph(self) -> None:
        assert self.obstacle_map is not None
        min_x, min_y, size_x, size_y = self.obstacle_map.grid.bbox
        X, Y = np.meshgrid(np.arange(min_x, min_x + size_x - GRID_RESOLUTION / 2, GRID_RESOLUTION),
                           np.arange(min_y, min_y + size_y, GRID_RESOLUTION * np.sqrt(3) / 2))
        X[::2] += GRID_RESOLUTION / 2

        rows, cols = self.obstacle_map.grid.to_grid(X.flatten(), Y.flatten())
        distance = ndimage.distance_transform_edt(1 - self.obstacle_map.map) * self.obstacle_map.grid.pixel_size
        D = ndimage.map_coordinates(distance, [[rows], [cols]], order=0).reshape(X.shape)
        gradient_y, gradient_x = np.gradient(distance)
        dD_dX = ndimage.map_coordinates(gradient_x, [[rows], [cols]], order=0).reshape(X.shape)
        dD_dY = ndimage.map_coordinates(gradient_y, [[rows], [cols]], order=0).reshape(X.shape)
        dD = np.sqrt(dD_dX**2 + dD_dY**2)
        close = np.logical_and(0.0 < D, D < MIN_MARGIN)
        close = np.logical_and(close, dD > 0)
        X[close] += dD_dX[close] / dD[close] * (MIN_MARGIN - D[close])
        Y[close] += dD_dY[close] / dD[close] * (MIN_MARGIN - D[close])

        keep = np.reshape([not all(self.obstacle_map.stack[int(np.round(row)), int(np.round(col)), :])
                           for row, col in zip(rows, cols, strict=True)], X.shape)
        keep[1::2, :] = np.logical_and(keep[1::2, :], D[1::2, :] < 2)
        keep[::4, 1::2] = np.logical_and(keep[::4, 1::2], D[::4, 1::2] < 2)
        keep[2::4, ::2] = np.logical_and(keep[2::4, ::2], D[2::4, ::2] < 2)
        self.tri_points = np.stack((X[keep], Y[keep]), axis=1)
        assert self.tri_points is not None  # NOTE: mypy doesn't seem to understand np.stack

        self.tri_mesh = spatial.Delaunay(self.tri_points)
        self.pose_groups = [
            DelaunayPoseGroup(
                index=i,
                point=Point(x=self.tri_points[i, 0], y=self.tri_points[i, 1]),
                neighbor_indices=_tri_neighbors(self.tri_mesh, i).tolist(),
                poses=[
                    Pose(x=point[0], y=point[1], yaw=np.arctan2(neighbor[1] - point[1], neighbor[0] - point[0]))
                    for neighbor in self.tri_points[_tri_neighbors(self.tri_mesh, i)]
                ],
            )
            for i, point in enumerate(self.tri_points)
        ]

        self.graph = nx.DiGraph()
        for g, group in enumerate(self.pose_groups):
            for p in range(len(group.poses)):
                self.graph.add_node((g, p))
        for g, group in enumerate(self.pose_groups):
            for p, (pose, g_) in enumerate(zip(group.poses, group.neighbor_indices, strict=True)):
                for p_, pose_ in enumerate(self.pose_groups[g_].poses):
                    if abs(angle(pose.yaw, pose_.yaw + np.pi)) < 0.01:
                        continue  # NOTE: avoid 180-degree turns
                    x, y, yaw = _generate_poses(self.obstacle_map.grid, pose, pose_)
                    if not self.obstacle_map.test(x, y, yaw).any():
                        length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
                        self.graph.add_edge((g, p), (g_, p_), backward=False, weight=length)
                        if ((g_, p_), (g, p)) not in self.graph.edges:
                            self.graph.add_edge((g_, p_), (g, p), backward=True, weight=1.2*length)

    def search(self, start: Pose, goal: Pose) -> list[PathSegment]:
        assert self.obstacle_map is not None
        assert self.graph is not None
        assert self.pose_groups is not None
        paths: list[list[PathSegment]] = []

        if TRY_SINGLE_PATH:
            for backward in [True, False]:
                simple_spline = Spline.from_poses(start, goal, backward=backward)
                if _is_healthy(simple_spline) and not self.obstacle_map.test_spline(simple_spline, backward):
                    paths.append([PathSegment(spline=simple_spline, backward=backward)])
        if paths:
            self.log.info('found single spline to reach goal')
            return min(paths, key=lambda path: path[0].spline.estimated_length())

        if TRY_SINGLE_SHUNTING:
            for backward in [True, False]:
                for length in [1, 1.5, 2]:
                    y_outline = [p[1] for p in self.robot_outline]
                    robot_length = max(y_outline) - min(y_outline)
                    step = PoseStep(linear=robot_length * (-length if backward else length), angular=0, time=0)
                    intermediate = start + step
                    spline1 = Spline.from_poses(start, intermediate, backward=backward)
                    if not _is_healthy(spline1) or self.obstacle_map.test_spline(spline1, backward):
                        continue
                    spline2 = Spline.from_poses(intermediate, goal, backward=not backward)
                    if not _is_healthy(spline2) or self.obstacle_map.test_spline(spline2, not backward):
                        continue
                    paths.append([
                        PathSegment(spline=spline1, backward=backward),
                        PathSegment(spline=spline2, backward=not backward),
                    ])
        if paths:
            self.log.info('found single shunt to reach goal')
            return min(paths, key=lambda path: path[0].spline.estimated_length() + path[1].spline.estimated_length())

        grid_entries = _find_grid_passages(self.obstacle_map, self.pose_groups, start, True)
        grid_exits = _find_grid_passages(self.obstacle_map, self.pose_groups, goal, False)
        if not grid_entries:
            raise RuntimeError('could not find start segment')
        if not grid_exits:
            raise RuntimeError('could not find exit segment')

        for enter, exit_ in itertools.product(grid_entries, grid_exits):
            p, g = enter.coordinate
            p_, g_ = exit_.coordinate
            path: list[PathSegment] = [enter.segment]
            last_g, last_p = g, p
            try:
                for next_g, next_p in nx.shortest_path(self.graph, (g, p), (g_, p_), weight='weight')[1:]:
                    last_pose = self.pose_groups[last_g].poses[last_p]
                    next_pose = self.pose_groups[next_g].poses[next_p]
                    backward = self.graph.edges[((last_g, last_p), (next_g, next_p))]['backward']
                    spline = Spline.from_poses(last_pose, next_pose, backward=backward)
                    path.append(PathSegment(spline=spline, backward=backward))
                    last_g, last_p = next_g, next_p
            except nx.exception.NetworkXNoPath:
                continue
            path.append(exit_.segment)

            while True:
                shortcuts: list[PathSegment] = []
                for step_size in [1, 2]:
                    for s in range(len(path) - step_size):
                        new_start = Pose(
                            x=path[s].spline.start.x,
                            y=path[s].spline.start.y,
                            yaw=path[s].spline.yaw(0) + (np.pi if path[s].backward else 0),
                        )
                        new_end = Pose(
                            x=path[s+step_size].spline.end.x,
                            y=path[s+step_size].spline.end.y,
                            yaw=path[s+step_size].spline.yaw(1) + (np.pi if path[s+step_size].backward else 0),
                        )
                        if abs(angle(new_start.yaw, new_end.yaw + np.pi)) < 0.01:
                            continue
                        for new_backward in [False, True]:
                            new_spline = Spline.from_poses(new_start, new_end, backward=new_backward)
                            if not _is_healthy(new_spline):
                                continue
                            if self.obstacle_map.test_spline(new_spline, new_backward):
                                continue
                            combined_length = path[s].spline.estimated_length() + \
                                path[s+step_size].spline.estimated_length()
                            if .9 * new_spline.estimated_length() > combined_length:
                                continue
                            shortcuts.append(PathSegment(spline=new_spline, backward=new_backward))
                        if shortcuts:
                            lengths = [segment.spline.estimated_length() for segment in shortcuts]
                            path[s] = shortcuts[np.argmin(lengths)]
                            for _ in range(step_size):
                                del path[s+1]
                            break  # restart while loop
                    if shortcuts:
                        break  # restart while loop
                if not shortcuts:
                    break  # exit while loop
            paths.append(path)
        if not paths:
            raise RuntimeError('could not find path')
        return min(paths, key=len)


def _tri_neighbors(tri_mesh: spatial.Delaunay, vertex_index: int) -> np.ndarray:
    return tri_mesh.vertex_neighbor_vertices[1][tri_mesh.vertex_neighbor_vertices[0][vertex_index]:
                                                tri_mesh.vertex_neighbor_vertices[0][vertex_index + 1]]


@lru_cache(maxsize=1000)
def _t_array(n: int) -> np.ndarray:
    return np.linspace(0, 1, n)


@lru_cache(maxsize=10000)
def _generate_pose_offsets(grid: Grid, dx: float, dy: float, yaw: float, yaw_: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    spline = FastSpline(0, 0, yaw, dx, dy, yaw_, False)
    row0, col0, layer0 = grid.to_3d_grid(0, 0, yaw)
    row1, col1, layer1 = grid.to_3d_grid(dx, dy, yaw_)
    num_rows = int(abs(row1 - row0))
    num_cols = int(abs(col1 - col0))
    num_layers = int(abs(layer1 - layer0))
    t = _t_array(max(num_rows, num_cols, num_layers))
    return (spline.x(t), spline.y(t), spline.yaw(t) + [0, np.pi][spline.backward])


def _generate_poses(grid: Grid, pose: Pose, pose_: Pose) -> tuple:
    dx, dy, yaw = _generate_pose_offsets(grid, pose_.x - pose.x, pose_.y - pose.y, pose.yaw, pose_.yaw)
    return pose.x + dx, pose.y + dy, yaw


def _is_healthy(spline: Spline, curvature_limit: float = 10.0) -> bool:
    return np.abs(spline.max_curvature()) < curvature_limit


@dataclass(slots=True, kw_only=True)
class Passage:
    segment: PathSegment
    coordinate: tuple[int, int]


def _find_grid_passages(obstacle_map: ObstacleMap,
                        pose_groups: list[DelaunayPoseGroup],
                        pose: Pose,
                        entering: bool,
                        max_num_groups: int = 10,
                        max_num_results: int = 3) -> list[Passage]:
    group_distances = [g.point.distance(pose) for g in pose_groups]
    group_indices = np.argsort(group_distances)
    results: list[Passage] = []
    for g, group in zip(group_indices, np.array(pose_groups)[group_indices][:max_num_groups], strict=False):
        for p, group_pose in enumerate(group.poses):
            for backward in [False, True]:
                poses = (pose, group_pose) if entering else (group_pose, pose)
                spline = Spline.from_poses(*poses, backward=backward)
                if _is_healthy(spline) and not obstacle_map.test_spline(spline, backward):
                    passage = Passage(segment=PathSegment(spline=spline, backward=backward), coordinate=(p, g))
                    results.append(passage)
    results.sort(key=lambda passage: passage.segment.spline.estimated_length())
    return results[:max_num_results]
