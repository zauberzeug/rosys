# World

The world object is the global data class which is constantly updated and changed by [Actors](actors.md).
To store your own data, we suggest you derive a project specific world from the one RoSys provides.

## Robot

The robots state is stored in `world.robot`.
Besides a [shape](user_interface.md#robot-and-shape) the object stores configuration parameters like maximum linear/angular speed, `minimum_turning_radius` and similar.
It also has a hardware configuration which will be translated to Lizard commands on startup.
There are three `Pose` objects describing current position and orientation of the robot:

1. **detection**: latest external referenced detection (obtained by GPS/RTK or cameras)
2. **prediction**: building on top of detection this pose is improved by all odometry information accumulated afterwards; this is the pose you normally use to determine where the robot is right now
3. **simulation**: the true pose of the robot without any noise (only used in simulation and testing)

## Notifications

Any notification scheduled by [Actors](actors.md#notifications) is stored in `world.notifications` as a list of two-value tuples.
The first value is the timestamp of the notification as float and the second contains the actual message.
Latest messages are at the end of the list.
