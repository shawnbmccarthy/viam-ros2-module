#
# this file is used to build out our virtual environment
# and add any environment variables needed to our run.sh
# script
#

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#sudo apt install python3-pip
#sudo apt-get install python3-venv

python3 -m venv ${SCRIPT_DIR}/venv

exec ${SCRIPT_DIR}/venv/bin/python -m pip install -r ${SCRIPT_DIR}/requirements.txt