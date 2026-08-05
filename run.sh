#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

# Let the container (running as root) draw on this session's X server.
# Harmless no-op if there's no X server (e.g. pure headless use).
xhost +local:docker >/dev/null 2>&1 || true

# ./run.sh                          -> interactive bash shell in the container
# ./run.sh colcon build             -> run a one-off command in the container
# ./run.sh ros2 launch sim_pkg ...  -> launch the sim directly
docker compose run --rm rc_sim "$@"
