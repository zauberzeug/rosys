#!/usr/bin/env bash

export COMPOSE_HTTP_TIMEOUT=200
docker-compose -f docker-compose-mac.yml "$@"