import numpy as np
from .grid import Grid
from .obstacle_map import ObstacleMap
from .robot_renderer import RobotRenderer


def generate_experiment(id_):
    group = abs(int(id_))
    variant = int(np.round(abs(id_) - group, 1) * 10)
    sign = 1 if id_ >= 0 else -1

    grid = Grid((60, 80, 36), (0.0, 0.0, 8.0, 6.0))

    robot_renderer = RobotRenderer.from_size(0.77, 1.21, 0.445)

    if group == 1:
        obstacles = [
            [1.5, 1.5, 0.2, 6.0],
            [1.5, 1.5, 5.0, 0.2],
            [6.3, 1.5, 0.2, 2.0],
            [4.5, 2.5, 0.2, 1.5],
            [5.5, 5.0, 0.2, 0.2],
        ]
        pose = (0.2, 0.75, 0)
        goal = (3.0, 3.5, 0) if variant == 0 else (0.75, 4.0, -np.pi / 2)
        obstacle_map = ObstacleMap.from_list(grid, obstacles, robot_renderer)
        small_obstacle_map = None

    if group == 2:
        obstacles = [
            [0.0, 2.0, 3.0, 0.2],
            [4.3, 2.0, 4.0, 0.2],
        ]
        pose = (0.5, 1.0, 0)
        goal = (3.6, 3.5, np.pi / 2)
        obstacle_map = ObstacleMap.from_list(grid, obstacles, robot_renderer)
        small_obstacle_map = None

    if group == 4:
        obstacles = [
            [5.0, 2.31, 3.0, 0.2],
            [5.0, 3.70, 3.0, 0.2],
        ]
        pose = (1.0, 3.1, 0)
        goal = (7.0, 3.1, np.pi)
        obstacle_map = ObstacleMap.from_list(grid, obstacles, robot_renderer)
        small_obstacle_map = None

    if group == 5:
        obstacles = [
            [4.0, 2.0, 2.0, 1.0],
            [4.0, 4.0, 2.0, 1.0],
        ]
        pose = (1.0, 1.0, 0 if variant == 0 else np.pi)
        goal = (6.0, 3.5, 0 if variant == 0 else np.pi)
        obstacle_map = ObstacleMap.from_list(grid, obstacles, robot_renderer)
        small_obstacle_map = None

    backward_to_goal = sign == -1

    return robot_renderer, pose, goal, obstacle_map, small_obstacle_map, backward_to_goal
