#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass

import networkx as nx
import numpy as np
import pylab as pl
from nicegui import ui
from scipy import ndimage, spatial

from rosys.actors.pathplanning import PlannerProcess
from rosys.actors.pathplanning import plot_tools as pt
from rosys.actors.pathplanning.robot_renderer import RobotRenderer
from rosys.helpers import angle
from rosys.world import Area, Obstacle, PathSegment, Point, Pose, Spline

seed = np.random.randint(0, 1000)
# seed = 300
ui.label(f'{seed=}')
print(f'{seed=}')
np.random.seed(seed)

robot_outline = [(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)]
areas = [Area(id='main', outline=[Point(x=-5, y=-5), Point(x=25, y=-5), Point(x=25, y=25), Point(x=-5, y=25)])]
obstacles = [
    Obstacle(id='0', outline=[Point(x=5, y=-5), Point(x=8, y=-5), Point(x=8, y=15), Point(x=5, y=15)]),
    Obstacle(id='1', outline=[Point(x=12, y=5), Point(x=15, y=5), Point(x=15, y=25), Point(x=12, y=25)]),
]
start = Pose(x=np.random.uniform(-4, 4), y=np.random.uniform(-4, 4), yaw=np.random.uniform(-np.pi, np.pi))
goal = Pose(x=np.random.uniform(16, 24), y=np.random.uniform(16, 24), yaw=np.random.uniform(-np.pi, np.pi))
planner = PlannerProcess(None, robot_outline)
plot = ui.plot(figsize=(14, 8))


def run_old():
    t = time.time()
    planner.update_obstacle_map(areas, obstacles, [start, goal], deadline=time.time()+10.0)
    planner.update_distance_map(goal, deadline=time.time()+10.0)
    dt0 = time.time() - t
    t = time.time()
    path = planner.search(goal=goal, start=start, backward=False, deadline=time.time()+10.0)
    dt1 = time.time() - t

    with plot:
        pl.clf()
        pl.title(f'{dt0:.3f} s + {dt1:.3f} s')
        pt.show_distance_map(planner.state.distance_map)
        pt.show_obstacle_map(planner.state.obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        pt.plot_path(path, 'C0')
        robot_renderer = RobotRenderer(robot_outline)
        pt.plot_robot(robot_renderer, (start.x, start.y, start.yaw), 'C2', lw=1)
        for step in path:
            yaw = step.spline.yaw(1) + np.pi if step.backward else step.spline.yaw(1)
            pt.plot_robot(robot_renderer, (step.spline.x(1), step.spline.y(1), yaw), 'C2', lw=1)


def run_new():
    t = time.time()
    planner.update_obstacle_map(areas, obstacles, [start, goal], deadline=time.time()+10.0)
    obstacle_map = planner.state.obstacle_map
    dt0 = time.time() - t

    t = time.time()

    min_x, min_y, size_x, size_y = obstacle_map.grid.bbox
    a = 1.0
    X, Y = np.meshgrid(
        np.arange(min_x, min_x + size_x - a / 2, a),
        np.arange(min_y, min_y + size_y, a * np.sqrt(3) / 2),
    )
    X[::2] += a / 2
    rows, cols = obstacle_map.grid.to_grid(X.flatten(), Y.flatten())
    keep = np.reshape(
        [not all(obstacle_map.stack[int(np.round(row)), int(np.round(col)), :]) for row, col in zip(rows, cols)],
        X.shape
    )
    distance = ndimage.distance_transform_edt(1 - obstacle_map.map) * obstacle_map.grid.pixel_size
    D = ndimage.map_coordinates(distance, [[rows], [cols]], order=0).reshape(X.shape)
    keep[1::2, :] = np.logical_and(keep[1::2, :], D[1::2, :] < 2)
    keep[::4, 1::2] = np.logical_and(keep[::4, 1::2], D[::4, 1::2] < 2)
    keep[2::4, ::2] = np.logical_and(keep[2::4, ::2], D[2::4, ::2] < 2)
    points = np.stack((X[keep], Y[keep]), axis=1)

    @dataclass
    class PoseGroup:
        index: int
        point: Point
        neighbor_indices: list[int]
        poses: list[Pose]

    tri = spatial.Delaunay(points)

    def neighbors(i: int): return tri.vertex_neighbor_vertices[1][tri.vertex_neighbor_vertices[0][i]:
                                                                  tri.vertex_neighbor_vertices[0][i + 1]]
    pose_groups: list[PoseGroup] = [
        PoseGroup(
            index=i,
            point=Point(x=points[i, 0], y=points[i, 1]),
            neighbor_indices=neighbors(i),
            poses=[
                Pose(x=point[0], y=point[1], yaw=np.arctan2(neighbor[1] - point[1], neighbor[0] - point[0]))
                for neighbor in points[neighbors(i)]
            ]
        )
        for i, point in enumerate(points)
    ]

    @dataclass
    class FastSpline:
        start_x: float
        start_y: float
        start_yaw: float
        end_x: float
        end_y: float
        end_yaw: float
        backward: bool

        def __post_init__(self):
            distance = -0.5 if self.backward else 0.5
            distance *= np.sqrt((self.end_x - self.start_x)**2 + (self.end_y - self.start_y)**2)
            self.a = self.start_x
            self.e = self.start_y
            self.b = self.start_x + distance * np.cos(self.start_yaw)
            self.f = self.start_y + distance * np.sin(self.start_yaw)
            self.c = self.end_x - distance * np.cos(self.end_yaw)
            self.g = self.end_y - distance * np.sin(self.end_yaw)
            self.d = self.end_x
            self.h = self.end_y
            self.m = self.d - 3 * self.c + 3 * self.b - self.a
            self.n = self.c - 2 * self.b + self.a
            self.o = self.b - self.a
            self.p = self.h - 3 * self.g + 3 * self.f - self.e
            self.q = self.g - 2 * self.f + self.e
            self.r = self.f - self.e

        @staticmethod
        def from_poses(start: Pose, end: Pose, *, backward: bool = False) -> FastSpline:
            return FastSpline(
                start_x=start.x,
                start_y=start.y,
                start_yaw=start.yaw,
                end_x=end.x,
                end_y=end.y,
                end_yaw=end.yaw,
                backward=backward,
            )

        def x(self, t: float) -> float:
            return t**3 * self.d + 3 * t**2 * (1 - t) * self.c + 3 * t * (1 - t)**2 * self.b + (1 - t)**3 * self.a

        def y(self, t: float) -> float:
            return t**3 * self.h + 3 * t**2 * (1 - t) * self.g + 3 * t * (1 - t)**2 * self.f + (1 - t)**3 * self.e

        def gx(self, t: float) -> float:
            return 3 * (self.m * t**2 + 2 * self.n * t + self.o)

        def gy(self, t: float) -> float:
            return 3 * (self.p * t**2 + 2 * self.q * t + self.r)

        def yaw(self, t: float) -> float:
            return np.arctan2(self.gy(t), self.gx(t))

        def create_poses(self) -> tuple:
            row0, col0, layer0 = obstacle_map.grid.to_grid(self.start_x, self.start_y, self.start_yaw)
            row1, col1, layer1 = obstacle_map.grid.to_grid(self.end_x, self.end_y, self.end_yaw)
            num_rows = int(abs(row1 - row0))
            num_cols = int(abs(col1 - col0))
            num_layers = int(abs(layer1 - layer0))
            n = max(num_rows, num_cols, num_layers)
            t = obstacle_map.t_lookup[n] if n < len(obstacle_map.t_lookup) else np.linspace(0, 1, n)
            return (self.x(t), self.y(t), self.yaw(t) + [0, np.pi][self.backward])

    G = nx.DiGraph()
    for g, group in enumerate(pose_groups):
        for p in range(len(group.poses)):
            G.add_node((g, p))
    for g, group in enumerate(pose_groups):
        for p, (pose, g_) in enumerate(zip(group.poses, group.neighbor_indices)):
            for p_, pose_ in enumerate(pose_groups[g_].poses):
                if abs(angle(pose.yaw, pose_.yaw + np.pi)) < 0.01:
                    continue  # NOTE: avoid 180-degree turns
                spline = FastSpline.from_poses(pose, pose_)
                x, y, yaw = spline.create_poses()
                if not obstacle_map.test(x, y, yaw).any():
                    length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
                    G.add_edge((g, p), (g_, p_), backward=False, weight=length)
                    if ((g_, p_), (g, p)) not in G.edges:
                        G.add_edge((g_, p_), (g, p), backward=True, weight=1.2*length)

    dt1 = time.time() - t

    t = time.time()

    def estimate_length(spline: Spline) -> float:
        dx = np.diff([spline.x(t) for t in np.linspace(0, 1, 10)])
        dy = np.diff([spline.y(t) for t in np.linspace(0, 1, 10)])
        return np.sum(np.sqrt(dx**2 + dy**2))

    def is_healthy(spline: Spline, curvature_limit: float = 10.0):
        return np.abs(spline.max_curvature()) < curvature_limit  # NOTE: max_curvature can be NaN

    def find_terminal_segment(terminal_pose: Pose, first: bool) -> tuple[PathSegment, int, int]:
        terminal_point = terminal_pose
        group_distances = [g.point.distance(terminal_point) for g in pose_groups]
        group_indices = np.argsort(group_distances)
        for g, group in zip(group_indices, np.array(pose_groups)[group_indices]):
            best_result = None
            best_length = np.inf
            for p, pose in enumerate(group.poses):
                for backward in [False, True]:
                    poses = (terminal_pose, pose) if first else (pose, terminal_pose)
                    spline = Spline.from_poses(*poses, backward=backward)
                    if is_healthy(spline) and not obstacle_map.test_spline(spline, backward):
                        length = estimate_length(spline)
                        if length < best_length:
                            best_length = length
                            best_result = (PathSegment(spline=spline, backward=backward), g, p)
            if best_result is not None:
                return best_result

    first_segment, g, p = find_terminal_segment(start, True)
    last_segment, g_, p_ = find_terminal_segment(goal, False)

    path: list[PathSegment] = []
    path.append(first_segment)
    last_g, last_p = g, p
    try:
        for next_g, next_p in nx.shortest_path(G, (g, p), (g_, p_), weight='weight')[1:]:
            last_pose = pose_groups[last_g].poses[last_p]
            next_pose = pose_groups[next_g].poses[next_p]
            backward = G.edges[((last_g, last_p), (next_g, next_p))]['backward']
            spline = Spline.from_poses(last_pose, next_pose, backward=backward)
            path.append(PathSegment(spline=spline, backward=backward))
            last_g, last_p = next_g, next_p
    except nx.exception.NetworkXNoPath:
        pass
    path.append(last_segment)

    dt2 = time.time() - t

    t = time.time()

    while True:
        for s in range(len(path) - 1):
            new_start = Pose(
                x=path[s].spline.start.x,
                y=path[s].spline.start.y,
                yaw=path[s].spline.yaw(0) + (np.pi if path[s].backward else 0),
            )
            new_end = Pose(
                x=path[s+1].spline.end.x,
                y=path[s+1].spline.end.y,
                yaw=path[s+1].spline.yaw(1) + (np.pi if path[s+1].backward else 0),
            )
            if abs(angle(new_start.yaw, new_end.yaw + np.pi)) < 0.01:
                continue
            new_backward = path[s+1].backward
            new_spline = Spline.from_poses(new_start, new_end, backward=new_backward)
            if not is_healthy(new_spline):
                continue
            if obstacle_map.test_spline(new_spline, new_backward):
                continue
            if .9 * estimate_length(new_spline) > estimate_length(path[s].spline) + estimate_length(path[s + 1].spline):
                continue
            path[s] = PathSegment(spline=new_spline, backward=new_backward)
            del path[s+1]
            break  # restart while loop
        else:
            break  # exit while loop

    dt3 = time.time() - t

    with plot:
        pl.clf()
        pl.title(f'map: {dt0:.3f} s, graph: {dt1:.3f} s, path: {dt2:.3f} s, smoothing: {dt3:.3f} s')
        pt.show_obstacle_map(obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        pl.triplot(points[:, 0], points[:, 1], tri.simplices, lw=0.1)

        pt.plot_path(path, 'C1')
        robot_renderer = RobotRenderer(robot_outline)
        pt.plot_robot(robot_renderer, (start.x, start.y, start.yaw), 'C0', lw=2)
        pt.plot_robot(robot_renderer, (goal.x, goal.y, goal.yaw), 'C0', lw=2)
        for step in path:
            for t in [0, 1]:
                yaw = step.spline.yaw(t) + np.pi if step.backward else step.spline.yaw(t)
                pt.plot_robot(robot_renderer, (step.spline.x(t), step.spline.y(t), yaw), 'C2', lw=1)


# run_old()
run_new()
with ui.row():
    ui.button('Run old', on_click=run_old)
    ui.button('Run new', on_click=run_new)

ui.run()
