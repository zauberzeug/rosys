#!/bin/bash
source /opt/ros/noetic/setup.bash

# Start roscore in the background
roscore &

# Wait for roscore to start
sleep 2

# Start the test image publisher in the background
python3 /test_image_publisher.py &

# Start the Foxglove bridge
roslaunch foxglove_bridge foxglove_bridge.launch 