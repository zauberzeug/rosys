# Rerun Visualization

RoSys integrates with [Rerun](https://rerun.io/) for advanced visualization of robot data including camera images, robot poses, and custom data over time.

## Overview

The Rerun integration provides:

- **Live streaming**: View camera images and robot poses in real-time
- **Timeline scrubbing**: Navigate through recorded data with Rerun's timeline controls
- **Multi-modal data**: Visualize images, 3D transforms, and other data types together
- **Recording**: Save sessions to `.rrd` files for later analysis

## Basic Example

```python
{! examples/rerun/main.py !}
```

## How It Works

1. **RerunLogger** starts a gRPC server for data streaming and an HTTP server for the web viewer
2. Connect data sources (cameras, odometer) to the logger
3. **RerunViewer** embeds the Rerun web viewer in your NiceGUI application
4. Data is automatically logged as it arrives and displayed in real-time

## Recording and Replay

To record a session:

```python
from pathlib import Path

# Start recording
rerun_logger.start_recording(Path('~/recordings/session.rrd'))

# ... robot operates ...

# Stop recording
rerun_logger.stop_recording()
```

To replay a recording, load the `.rrd` file in the viewer:

```python
viewer.load_recording('/recordings/session.rrd')
```

## Ports

By default, the Rerun integration uses:

- **Port 9876**: gRPC server for data streaming
- **Port 9877**: HTTP server for the web viewer

You can customize these in the `RerunLogger` constructor:

```python
rerun_logger = RerunLogger(
    app_id='my_robot',
    grpc_port=9876,
    web_port=9877,
)
```

## Custom Data

You can log custom data using the `log_custom` method:

```python
import rerun as rr

# Log 3D points
rerun_logger.log_custom('sensors/lidar', rr.Points3D(points))

# Log a text annotation
rerun_logger.log_custom('status', rr.TextLog('Robot initialized'))
```

See the [Rerun documentation](https://rerun.io/docs) for all available archetypes.
