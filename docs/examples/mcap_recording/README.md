# MCAP Recording

This example records simulated robot data to MCAP files for replay and analysis in Foxglove Studio.

After starting the application, the developer panel offers start/stop buttons and a topic selection, and the recordings can be listed and downloaded at http://localhost:8080/recordings.

```python hl_lines="16-18 20"
{! examples/mcap_recording/main.py !}
```

Recorder
: The `McapRecorder` writes every registered topic to rotating MCAP files under `~/.rosys/mcap`.
With `auto_start=False` it stays idle until you press the record button in `developer_ui`.

Topics
: `add_event_topic` and `add_pose_topic` bind a topic to a RoSys event; the matching Foxglove converter is picked from the payload type automatically.
The subscription is only active while a recording is open.

Recordings page
: `RecordingsPage` mounts a page for listing, renaming, reindexing and downloading recordings, plus a download endpoint at `/api/recordings/{name}`.
The optional `header` callback renders shared navigation at the top of the page.
