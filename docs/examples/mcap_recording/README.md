# MCAP Recording

This example records simulated robot data to MCAP files for replay and analysis in Foxglove Studio.

After starting the application, the developer panel offers start/stop buttons and a topic selection, and the recordings can be listed and downloaded at http://localhost:8080/recordings.

```python hl_lines="16-18 20"
{! examples/mcap_recording/main.py !}
```

Recorder
: The `McapRecorder` writes every registered topic to rotating MCAP files under `~/.rosys/mcap`.
With `auto_start=False` it stays idle until you press the record button in `developer_ui`.

Runs and parts
: Each recording is a run named after its start time, and its files are numbered parts: `<run>_01.mcap`, `<run>_02.mcap`, ...
A part is rotated by size (`max_file_size_mb`) and, with `max_file_duration`, by age.
`recorder.start(name='mission', metadata={'field': 'north'})` names the run `<timestamp>_mission`, writes the metadata into every part and returns the run's name.

Disk budget
: Before a part is opened, the oldest recordings are deleted to stay within `max_total_size_mb`, the recorder's own parts first.
Renamed and merged recordings count as kept and have a bound of their own, `max_kept_size_mb`, which spares the newest kept file.

Topics
: `add_event_topic` and `add_pose_topic` bind a topic to a RoSys event; the matching Foxglove converter is picked from the payload type automatically.
The subscription is only active while a recording is open.

Image quality
: Camera topics dominate a recording's size.
Keyword arguments are forwarded to the converter, so `add_event_topic(recorder, '/camera/front/image', event=camera.NEW_IMAGE, quality=75)` records JPEGs at quality 75 instead of the default 90, roughly halving the bytes per frame.

Log lines
: `logger.addHandler(McapLogHandler(recorder))` records what a logger logs on the `/log` topic, so Foxglove shows the log next to the data it explains.

Merging
: `await recorder.merge(parts, 'mission_merged', start_time_ns=...)` merges finished recordings into one kept `mission_merged.mcap` and deletes the parts once it is in place.
Messages before `start_time_ns` are dropped, and the merged file keeps the metadata of every run it holds.

Recordings page
: `RecordingsPage` mounts a page for listing, renaming, reindexing, merging and downloading recordings, plus a download endpoint at `/api/recordings/{name}`.
The parts of a run are listed as one entry that can be merged with a click.
The optional `header` callback renders shared navigation at the top of the page.
