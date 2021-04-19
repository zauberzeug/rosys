# Robot Brain

## Continuous Build

We build with Drone. Run locally with

    drone starlark .drone.star && drone exec --trusted --exclude slack  .drone.yml
