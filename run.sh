#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Create virtual Python environment
cd `dirname $0`
python3 -m venv --system-site-packages venv

./venv/bin/python3 -m pip install -r requirements.txt

# export underlay & add overlays as needed
. /etc/turtlebot4/setup.bash
. /opt/ros/humble/setup.bash

# setup LD_LIBRARY_PATH for viam
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/python3.10/dist-packages/viam/rpc/:${SCRIPT_DIR}/venv/lib/python3.10/site-packages/viam/rpc/

# TODO: ctrl-c seems to kill the run.sh script while leaving the child process running
exec ${SCRIPT_DIR}/venv/bin/python3 ${SCRIPT_DIR}/src/main.py $@
