from nicegui import ui
import rosys
import rosys.ui

shape = rosys.RobotShape(outline=[
    (0, 0), (-0.5, -0.5), (1.5, -0.5), (1.75, 0), (1.5, 0.5), (-0.5, 0.5),
])  # the shape for the robot will be used in 3d rendering
world = rosys.World(mode=rosys.Mode.SIMULATION, robot=rosys.Robot(shape=shape))
runtime = rosys.Runtime(world)
rosys.ui.configure(ui, runtime)

with ui.scene() as scene:
    # by passing `debug=True` to the robot 3d object you will see the wireframe, axis-center and follow-the-line target
    robot = rosys.ui.robot_object(debug=True)

ui.run(port=8080)
