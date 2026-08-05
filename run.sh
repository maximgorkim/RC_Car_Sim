#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

xhost +local:docker >/dev/null 2>&1 || true

docker compose up -d --build rc_sim

if [ "$#" -eq 0 ]; then
    docker compose exec rc_sim bash
else
    docker compose exec rc_sim bash -lc '
        source /opt/ros/humble/setup.bash

        cd /root/bfmc_sim

        if [ -f install/setup.bash ]; then
            source install/setup.bash
        fi

        exec "$@"
    ' bash "$@"
fi