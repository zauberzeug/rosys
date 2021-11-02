#!/usr/bin/env bash

if [[ $1 = "debug" ]]; then
   nice -n -19 python3 -m debugpy --listen 5678 main.py
else
   nice -n -19 python3 main.py
fi
