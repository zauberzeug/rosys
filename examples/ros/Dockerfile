FROM ros:noetic-ros-base

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install nicegui pyserial

SHELL ["/bin/bash", "-c"]

COPY CMakeLists.txt /root/rosys/catkin_ws/src/robot/
COPY package.xml /root/rosys/catkin_ws/src/robot/

RUN . /opt/ros/noetic/setup.bash && \
    cd /root/rosys/catkin_ws && \
    catkin_make && \
    . devel/setup.bash

RUN echo "#!/bin/bash" > /ros_entrypoint.sh && \
    echo "source /root/rosys/catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh && \
    echo "exec \"\$@\"" >> /ros_entrypoint.sh

ADD scripts /root/rosys/catkin_ws/src/robot/scripts
ADD launch /root/rosys/catkin_ws/src/robot/launch
COPY lizard.txt /root/rosys/catkin_ws/src/robot/

CMD [ "roslaunch", "/root/rosys/catkin_ws/src/robot/launch/robot.launch" ]
