FROM ros:noetic-ros-base

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install nicegui

WORKDIR /app/

CMD roscore
