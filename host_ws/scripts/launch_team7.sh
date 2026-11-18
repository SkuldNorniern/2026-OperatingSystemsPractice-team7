#!/usr/bin/env bash
# usage: launch_team7.sh [same options as run_demo.sh]   team7 run on the robot, replay or stock build
exec "$(dirname "${BASH_SOURCE[0]}")/run_demo.sh" "$@"
