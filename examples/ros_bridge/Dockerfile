FROM ros:noetic-perception-focal

# Install required packages
RUN apt-get update && apt-get install -y \
    ros-noetic-foxglove-bridge \
    ros-noetic-cv-bridge \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Copy the scripts
COPY launch.sh /launch.sh
COPY test_image_publisher.py /test_image_publisher.py
RUN chmod +x /launch.sh /test_image_publisher.py

# Set the entrypoint
ENTRYPOINT ["/launch.sh"]