#!/usr/bin/env python3
import time
from dataclasses import dataclass

import networkx as nx
import numpy as np
import pylab as pl
from nicegui import ui
from scipy import spatial

from rosys.actors.pathplanning import PlannerProcess
from rosys.actors.pathplanning import plot_tools as pt
from rosys.actors.pathplanning.robot_renderer import RobotRenderer
from rosys.actors.pathplanning.steps import Path, Step
from rosys.world import Area, Obstacle, PathSegment, Point, Pose, Spline

robot_outline = [(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)]
areas = [Area(id='main', outline=[Point(x=-5, y=-5), Point(x=25, y=-5), Point(x=25, y=25), Point(x=-5, y=25)])]
obstacles = [Obstacle(id='0', outline=[Point(x=5, y=-5), Point(x=15, y=-5), Point(x=15, y=15), Point(x=5, y=15)])]
start = Pose()
goal = Pose(x=20, y=0, yaw=0)
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
    planner.update_distance_map(goal, deadline=time.time()+10.0)
    dt0 = time.time() - t

    t = time.time()

    min_x, min_y, size_x, size_y = planner.state.obstacle_map.grid.bbox
    points = []
    while True:
        for _ in range(100):
            planner.state.obstacle_map.grid.bbox
            point = Point(x=min_x + size_x * np.random.rand(), y=min_y + size_y * np.random.rand())
            row, col = planner.state.obstacle_map.grid.to_grid(point.x, point.y)
            if all(planner.state.obstacle_map.stack[int(np.round(row)), int(np.round(col)), :]):
                continue
            if any(p.distance(point) < 2.0 for p in points):
                continue
            points.append(point)
            break
        else:
            break
    points = np.array([[p.x, p.y] for p in points])

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

    G = nx.Graph()
    for g, group in enumerate(pose_groups):
        for p in range(len(group.poses)):
            G.add_node((g, p))
    for g, group in enumerate(pose_groups):
        for p, (pose, g_) in enumerate(zip(group.poses, group.neighbor_indices)):
            for p_, pose_ in enumerate(pose_groups[g_].poses):
                spline = Spline.from_poses(pose, pose_)
                if not planner.state.obstacle_map.test_spline(spline):
                    G.add_edge((g, p), (g_, p_))

    dt1 = time.time() - t

    t = time.time()

    def find_first_segment() -> tuple[PathSegment, int, int]:
        start_point = start.point
        group_distances = [g.point.distance(start_point) for g in pose_groups]
        group_indices = np.argsort(group_distances)
        for g, group in zip(group_indices, np.array(pose_groups)[group_indices]):
            for p, pose in enumerate(group.poses):
                for backward in [False, True]:
                    spline = Spline.from_poses(start, pose, backward=backward)
                    if not planner.state.obstacle_map.test_spline(spline):
                        return PathSegment(spline=spline, backward=backward), g, p

    def find_last_segment() -> tuple[PathSegment, int, int]:
        goal_point = goal.point
        group_distances = [g.point.distance(goal_point) for g in pose_groups]
        group_indices = np.argsort(group_distances)
        for g, group in zip(group_indices, np.array(pose_groups)[group_indices]):
            for p, pose in enumerate(group.poses):
                for backward in [False, True]:
                    spline = Spline.from_poses(pose, goal, backward=backward)
                    if not planner.state.obstacle_map.test_spline(spline):
                        return PathSegment(spline=spline, backward=backward), g, p

    first_segment, g, p = find_first_segment()
    last_segment, g_, p_ = find_last_segment()

    path: list[PathSegment] = []
    path.append(first_segment)
    last_pose = pose_groups[g].poses[p]
    for path_g, path_p in nx.shortest_path(G, (g, p), (g_, p_))[1:]:
        next_pose = pose_groups[path_g].poses[path_p]
        spline = Spline.from_poses(last_pose, next_pose, backward=False)
        path.append(PathSegment(spline=spline, backward=False))
        last_pose = next_pose
    path.append(last_segment)

    steps = Path()
    last_step = None
    for segment in path:
        pose = segment.spline.pose(1)
        step = Step((pose.x, pose.y, pose.yaw), last_step)
        steps.append(step)
        last_step = step
    steps.smooth(planner.state.obstacle_map)

    dt2 = time.time() - t

    with plot:
        pl.clf()
        pl.title(f'{dt0:.3f} s + {dt1:.3f} s + {dt2:.3f} s')
        pt.show_distance_map(planner.state.distance_map)
        pt.show_obstacle_map(planner.state.obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        pl.triplot(points[:, 0], points[:, 1], tri.simplices, lw=0.1)
        for group in pose_groups:
            for pose in group.poses:
                pl.plot([pose.x, pose.x + np.cos(pose.yaw)], [pose.y, pose.y + np.sin(pose.yaw)], 'C2-')
        for (g, p), (g_, p_) in G.edges:
            pl.plot([pose_groups[g].poses[p].x, pose_groups[g_].poses[p_].x],
                    [pose_groups[g].poses[p].y, pose_groups[g_].poses[p_].y], 'C0-', lw=1)

        pt.plot_path(path, 'C0')
        robot_renderer = RobotRenderer(robot_outline)
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
