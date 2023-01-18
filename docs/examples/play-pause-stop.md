# Play-pause-stop

In this example, we use the `AutomationControls` UI element to start, pause and stop an automation.
Here we let the robot drive to predefined checkpoints one after the other.

```python
{!src/example_play-pause-stop.py!}
```

To achieve this, we define our automation as an async method and pass it to the `default_automation` parameter of the `Automator`.

![Click-and-drive](play-pause-stop.webp){: style="width:80%"}
