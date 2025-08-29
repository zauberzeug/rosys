# Automations

This example demonstrates how to start, pause, resume and stop automations.
The robot will drive forward and then turn left in an endless loop.
The `turn_left` function is marked with `@rosys.automation.uninterruptible` which means that it will continue to run even if the automation is interrupted by pausing or stopping it.

```python
{! examples/automations/main.py !}
```
