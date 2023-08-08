#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# export underlay & add overlays as needed
. /opt/ros/humble/setup.bash

# this assumes viam is installed globally for ROS2 robots
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/python3.10/dist-packages/viam/rpc/

# TODO: ctrl-c seems to kill the run.sh script while leaving the child process running
exec python3 ${SCRIPT_DIR}/main.py $@
