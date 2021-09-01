import numpy as np
from scipy import ndimage
import cv2


class ObstacleMap:

    def __init__(self, grid, map_, robot_shape):
        self.grid = grid
        self.map = map_
        self.stack = np.zeros(grid.size, dtype=bool)
        self.dist_stack = np.zeros(self.stack.shape)
        for layer in range(grid.size[2]):
            _, _, yaw = grid.from_grid(0, 0, layer)
            kernel = robot_shape.render(grid.pixel_size, yaw).astype(np.uint8)
            self.stack[:, :, layer] = cv2.dilate(self.map.astype(np.uint8), kernel)
            self.dist_stack[:, :, layer] = \
                ndimage.distance_transform_edt(~self.stack[:, :, layer]) * grid.pixel_size

        # NOTE: when yaw wraps around, map_coordinates should wrap around on axis 2
        self.stack = np.dstack((self.stack, self.stack[:, :, :1]))
        self.dist_stack = np.dstack((self.dist_stack, self.dist_stack[:, :, :1]))

    @staticmethod
    def from_list(grid, obstacles, robot_shape):
        map_ = np.zeros(grid.size[:2], dtype=bool)
        for x, y, w, h in obstacles:
            r0, c0 = grid.to_grid(x, y)
            r1, c1 = grid.to_grid(x + w, y + h)
            map_[int(np.round(r0)):int(np.round(r1))+1,
                 int(np.round(c0)):int(np.round(c1))+1] = True
        return ObstacleMap(grid, map_, robot_shape)

    def test(self, x, y, yaw):
        row, col, layer = self.grid.to_grid(x, y, yaw)
        return ndimage.map_coordinates(self.stack, [[row], [col], [layer]], order=0)

    t_lookup = [np.linspace(0, 1, i) for i in range(360)]

    def _create_poses(self, spline, backward):
        def pose(t): return (
            spline.x(t),
            spline.y(t),
            spline.yaw(t) + [0, np.pi][backward],
        )
        row0, col0, layer0 = self.grid.to_grid(*pose(0.0))
        row1, col1, layer1 = self.grid.to_grid(*pose(1.0))
        num_rows = int(abs(row1 - row0))
        num_cols = int(abs(col1 - col0))
        num_layers = int(abs(layer1 - layer0))
        n = max(num_rows, num_cols, num_layers)
        t = ObstacleMap.t_lookup[n] if n < len(ObstacleMap.t_lookup) else np.linspace(0, 1, n)
        return pose(t)

    def test_spline(self, spline, backward=False):
        return self.test(*self._create_poses(spline, backward)).any()

    def get_distance(self, x, y, yaw):
        row, col, layer = self.grid.to_grid(x, y, yaw)
        return ndimage.map_coordinates(self.dist_stack, [[row], [col], [layer]], order=0)

    def get_minimum_spline_distance(self, spline, backward=False):
        return self.get_distance(*self._create_poses(spline, backward)).min()
