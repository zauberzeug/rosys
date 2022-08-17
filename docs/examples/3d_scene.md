# 3D Scene

## Robot and Shape

It is often desired to visualize all the robot's information about the world.
To do so you can create a 3d scene with [NiceGUI](https://nicegui.io).
RoSys provides a `robot_object` to render and update the robot:

```python hl_lines="10-12"
{!src/robot_shape.py [ln:5-16] !}
```

## Click Handler

You can also pass a click handler to the 3d scene.
Here is a full example example for driving to a point on the ground by starting the built-in automation called `drive_to`:

```python hl_lines="12 18"
{!src/scene_on_click.py!}
```

![Click Handler](3d_scene_click_handler.webp){: style="width:60%"}
