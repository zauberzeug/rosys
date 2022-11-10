# Simulation Speed

When running in simulation you can accelerate the time.
Here we have set up a fence in which the robot moves to random positions.
With a simple slider the execution time is accelerated.
Note how the time advances faster it the simulation speed is increased.
The driving speed of the robot remains the same.

![simulation speed](simulation_speed.webp){: style="width:60%"}

This is archived simply by placing `rosys.simulation_ui()` in your UI.
The rest of the code is needed to define the boundary, draw it in the 3d scene and start the automation for random movement:

```python hl_lines="29"
{!src/example_simulation_speed.py!}
```
