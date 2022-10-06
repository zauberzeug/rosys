#!/usr/bin/env bash
if [[ $1 = "debug" ]]; then
   nice -n -5 python3 -m debugpy --listen 5678 /app/main.py
elif [[ $1 = "profile" ]]; then
   nice -n -5 py-spy record -o profile.svg -- python3 /app/main.py
else
   nice -n -5 python3 /app/main.py
fi
