#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export DISPLAY=:1
exec python3 "$HOME/robot_ws/src/rda_robot_assembler/rda_robot_assembler/app.py" "$@"
