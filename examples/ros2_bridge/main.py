#!/usr/bin/env python3
from pathlib import Path
from typing import Any, Callable, Optional
import base64
import json
import time

import cv2
import numpy as np
from nicegui import ui
import roslibpy

import rosys


class RosBridge():
    def __init__(self, websocket_url: str = "ws://localhost:9090") -> None:
        """Initialize ROS bridge with websocket connection.

        Args:
            websocket_url: WebSocket URL for ROS bridge connection (default uses port 9090)
        """
        host = websocket_url.split('://')[1].split(':')[0]
        port = int(websocket_url.split(':')[-1])
        
        self.client = roslibpy.Ros(host=host, port=port)
        self._image_callback: Callable | None = None
        self._connected = False

        # Ensure images directory exists
        self.image_dir = Path('images')
        self.image_dir.mkdir(exist_ok=True)

        rosys.on_repeat(self.connect, 0.1)

    async def run(self) -> None:
        while True:
            if not self._connected:
                await self.connect()
                await rosys.sleep(0.1)
                continue
            await rosys.sleep(0.1)

    async def connect(self, max_attempts: int = 3, retry_delay: int = 2) -> bool:
        """Connect to ROS bridge websocket server with retries."""
        if self._connected:
            return True

        for attempt in range(max_attempts):
            try:
                self.client.run()
                self._connected = True
                print("Connected to ROS bridge websocket server")
                return True
            except Exception as e:
                if attempt < max_attempts - 1:
                    print(f"Connection attempt {attempt + 1} failed. Retrying in {retry_delay} seconds...")
                    print(f"Error: {e}")
                else:
                    print("Error: Could not connect to ROS bridge websocket server")
                    print("Please make sure:")
                    print("1. The ROS2 bridge container is running")
                    print("2. The container has network access")
                    print("3. No other service is using the port")
        return False

    def _handle_image(self, message) -> None:
        """Internal handler for image messages."""
        try:
            # Extract timestamp from ROS2 message
            if 'header' in message and 'stamp' in message['header']:
                # Combine seconds and nanoseconds for precise timestamp
                secs = int(message['header']['stamp']['sec'])
                nsecs = int(message['header']['stamp']['nanosec'])
                # Format timestamp properly with full nanosecond precision
                ts = secs + nsecs / 1e9
            else:
                # Fallback to current time if no timestamp
                ts = time.time()
            
            # Get image data
            encoding = message.get('encoding', 'bgr8')
            height = message['height']
            width = message['width']
            step = message['step']
            
            # Decode image data
            img_data = base64.b64decode(message['data'])
            img_array = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, 3)

            # Format timestamp string with nanosecond precision
            timestamp_str = f"{ts:.9f}"  # Show all 9 decimal places
            
            # Save as JPEG with full timestamp
            image_path = self.image_dir / f"{timestamp_str}.jpg"
            is_success, buffer = cv2.imencode('.jpg', img_array)

            if is_success:
                image_path.write_bytes(buffer.tobytes())
                print(f"Saved image at timestamp {timestamp_str}")
            else:
                print("Failed to encode image")

            # Call user callback if set
            if self._image_callback:
                self._image_callback(img_array, float(timestamp_str))

        except Exception as e:
            print(f"Error processing image message: {e}")
            print(f"Message structure: {json.dumps(message, indent=2)}")

    def subscribe_to_images(self, topic: str, callback: Callable | None = None) -> None:
        """Subscribe to ROS image topic.

        Args:
            topic: ROS topic name
            callback: Optional callback function(image_array, timestamp)
        """
        if not self._connected:
            raise RuntimeError("Not connected to ROS bridge. Call connect() first")

        self._image_callback = callback
        listener = roslibpy.Topic(self.client, topic, 'sensor_msgs/Image')
        listener.subscribe(self._handle_image)
        print(f"Subscribed to image topic: {topic}")

    def close(self) -> None:
        """Close the connection."""
        if self.client:
            self.client.terminate()
        self._connected = False


class FileCounter(ui.label):
    def __init__(self) -> None:
        super().__init__()
        self.count = 0
        rosys.on_repeat(self.count_files, 1)

    def count_files(self) -> None:
        # Count files in images directory
        image_files = len(list(Path('./images').glob('*.jpg')))
        self.count = image_files
        self.text = f"Files: {self.count}"


def on_image(image_array: np.ndarray, timestamp: float) -> None:
    print(f"Custom handler: Received image at {timestamp}")


ros_bridge = RosBridge()


def subscribe_to_images():
    ros_bridge.subscribe_to_images('/test_image', callback=on_image)


ui.button('Subscribe to images', on_click=subscribe_to_images)

FileCounter()
ui.run(title='Camera Arm') 