# Automations

RoSys is inherently designed to be used for autonomous robots.
The Automator [actor](actors.md) provides an easy way to write logical step-wise rules for this.

## Pausing

To pause an automation you should fire the event `event.Id.PAUSE_AUTOMATIONS` and provide an explanation as string parameter.
The runtime provides an handy wrapper `runtime.pause(because='...')`.
