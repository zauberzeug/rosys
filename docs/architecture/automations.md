# Automations

RoSys is inherently designed to be used for autonomous robots.
The automator [actor](actors.md) provides an easy way to write logical step-wise rules for this.

## Automation Controls

RoSys provides ready-made UI elements to start, pause, resume and stop automations.
See [Automation Controls in the User Interface section](user_interface.md#automation-controls) for an example.

## Default Automation

In most cases your robot will have one automation which describes its automatic behavior.
This automation can be passed to `rosys.ui.automation_controls` in your main file.
When the user presses the start button, RoSys will begin executing this default automation.

## Pausing

To pause the ongoing automation you should fire the event `event.Id.PAUSE_AUTOMATION` and provide an explanation as string parameter.
The runtime provides a handy wrapper `runtime.automation.pause(because='...')`.
From within an [actor](actors.md) you can use `self.pause_automation(because='...')`.
