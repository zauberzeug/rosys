# Automation Controls

Required for [safety](../safety.md) and good usability: automation processes only begin after the user an actively requests it.
Also the user should always be able to pause/resume and stop an ongoing automation.
While you could write your own UI, RoSys already provides a ready-made set of elements with `rosys.ui.automation_controls()`.
Building on the [click handler from the 3d scene](3d_scene.md#click-handler) example we can add these easily:

```python hl_lines="3"
{!src/scene_on_click_with_automation_controls.py [ln:21-23]!}
```

The `ui.row()` context arranged the control buttons in a row.
