from ...world.point import Point
from ...world.pose import Pose
from ...world.spline import Spline


class Carrot:

    def __init__(self, spline: Spline):

        self.spline = spline
        self.t = 0
        self.is_end_reached = False

    @property
    def pose(self) -> Pose:

        if self.t < 1.0:
            return self.spline.pose(self.t)
        else:
            return Pose(
                x=self.spline.x(1.0) + (self.t - 1.0) * self.spline.gx(1.0),
                y=self.spline.y(1.0) + (self.t - 1.0) * self.spline.gy(1.0),
                yaw=self.spline.yaw(1.0),
            )

    def move(self, point_of_interest: Point, distance: float = 1.0, move_threshold: float = 0.01):

        end_pose = self.spline.pose(1.0)
        end_point = end_pose.point
        end_yaw = end_pose.yaw

        while point_of_interest.distance(self.pose.point) < distance:

            self.t += 0.01

            if self.t < 1.0:
                continue

            if point_of_interest.projected_distance(end_point, end_yaw) <= move_threshold:
                return False

        return True
