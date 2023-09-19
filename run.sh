#!/bin/bash

#
# this file is used to build out our virtual environment
# and add any environment variables needed to our run.sh
# script
#
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#
# setup of virtual environment
# TODO: create flag vs. update flag
#
if [ ! -d ${SCRIPT_DIR}/venv/bin/python ]; then
  echo "Setting up virtual environment"
  python3 -m venv ${SCRIPT_DIR}/venv
  exec ${SCRIPT_DIR}/venv/bin/python -m pip install -r ${SCRIPT_DIR}/requirements.txt
else
  echo "virtual environment exists"
fi
export VIAM_ROS_NODE_NAME=
export ROS_NAMESPACE=

#
# export underlay & add overlays as needed
# TODO: Update environment based on requirements for ROS Overlays
#
. /etc/viam/setup.bash
#. /etc/turtlebot4/setup.bash
#. /opt/ros/humble/setup.bash

# setup LD_LIBRARY_PATH for viam
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/python3.10/dist-packages/viam/rpc/:${SCRIPT_DIR}/venv/lib/python3.10/site-packages/viam/rpc/

# TODO: ctrl-c seems to kill the run.sh script while leaving the child process running
exec ${SCRIPT_DIR}/venv/bin/python3 ${SCRIPT_DIR}/src/main.py $@
