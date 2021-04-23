from pydantic import BaseModel
from world.pose import Pose
from world.velocity import Velocity


class Robot(BaseModel):

    pose: Pose = Pose()
    velocity: Velocity = Velocity()
