#!/usr/bin/env bash

if [[ $1 = "debug" ]]; then
   python3 -m debugpy --listen 5678 /app/main.py
elif [[ $1 = "profile" ]]; then
    kernprof /app/main.py
else
    # only use one worker because we do not have a message quque (https://trello.com/c/qpgsAx9y/46-deployment-auf-learning-loopai#comment-5fe6d7d9ef84285e09cd10dc)
    export MAX_WORKERS=1
    gunicorn -k "uvicorn.workers.UvicornWorker" -c /gunicorn_conf.py "main:app"
fi