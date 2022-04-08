# Events

RoSys provides an integrated event bus to chain otherwise separated parts of the system together.
This allows loosely coupled actors where each one has its own dedicated role in a more complex workflow.
For example the Lizard actor repeatedly reads the machine data like the current velocity, writes them into the world and finally emits the event `NEW_MACHINE_DATA`.
The Odometer actor has registered itself for this event to compute the new position of the robot.
Other actors may also register for the same event to monitor battery level or bump events.

## Example

Implement a geofence actor pausing the automation when the robot position exceeds a certain threshold:

```python
{!src/geofence.py!}
```

![Geofence](geofence.gif)
