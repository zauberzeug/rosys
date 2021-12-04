# Automations

RoSys is inherently designed to be used for autonomous robots.
The Automator [actor](actors.md) provides an easy way to write logical step-wise rules for this.

## Automation Controls

RoSys provides ready-made UI elements to start, pause/resume and stop automations.
See [Automation Controls in the User Interface section](user_interface.md#automation-controls) for an example.

## Default Automation

In most cases your robot will have one automation which describes it's automatic behavior.
This automation should be set as `runtime.automator.default_automation` in your main file.
When the user signals RoSys that it is ok to start the automations[^1], it will began executing this default automation.

[^1]:
    Normally the signal comes from an UI component like the [Automation Controls](automations.md#automation-controls).
    See [Safety](safety.md) for more information.

## Pausing

To pause the ongoing automations you should fire the event `event.Id.PAUSE_AUTOMATIONS` and provide an explanation as string parameter.
The runtime provides an handy wrapper `runtime.pause(because='...')`.
From within an [Actor](actors.md) you can use `self.pause_automations(because='...')`.
