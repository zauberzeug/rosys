# Robot Brain

## Starting

### Jetson

The `docker-compose.yml` (eg. default compose file name) defines the configuration for the infrastructure running on Robot Brain (Nvidia Jetson).

1. define `ESP_SERIAL` in the `.env` file (see `z3.env` for an example)
2. launch with `docker-compose up -d --build`
3. on reboot or crash the services will start automatically

### Mac

We use a seperate `docker-compose-mac.yml` on Mac because we can not use the host network here. To simplify usage we wrapped this in a script:

    ./compose.sh up -d --build

## Continuous Build

We build with Drone. Run locally with

    drone starlark .drone.star && drone exec --trusted --exclude slack  .drone.yml
