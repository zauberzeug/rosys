from pydantic import BaseModel
from typing import Optional
from ...world.point import Point
from ...world.pose import Pose
from ...world.spline import Spline


class Carrot(BaseModel):

    spline: Spline
    t: float = 0
    target_distance: Optional[float] = None

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

            self.target_distance = point_of_interest.projected_distance(end_point, end_yaw)
            if self.target_distance <= move_threshold:
                return False

        return True
