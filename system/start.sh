#!/usr/bin/env bash

uvicorn main:app --host 0.0.0.0 --port 80 --reload --lifespan on --forwarded-allow-ips='*' --proxy-headers