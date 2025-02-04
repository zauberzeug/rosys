#!/bin/bash
source /opt/ros/humble/setup.bash

# Start the test image publisher in the background
python3 /test_image_publisher.py &

# Start the ROS bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 