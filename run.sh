#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
. /opt/ros/humble/setup.bash

# TODO: ctrl-c seems to kill the run.sh script while leaving the child process running
${SCRIPT_DIR}/venv/bin/python3 ${SCRIPT_DIR}/main.py $@
