from world.pose import Pose


class Carrot:

    def __init__(self, spline):

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

    def move(self, robot_pose, distance=1.0, move_threshold=0.01):

        while robot_pose.distance(self.pose) < distance:

            self.t += 0.01

            if self.t < 1.0:
                continue

            if robot_pose.projected_distance(self.spline.end) <= move_threshold:
                return False

        return True
