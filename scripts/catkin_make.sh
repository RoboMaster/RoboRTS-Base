#!/bin/bash
CATKIN_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." >/dev/null 2>&1 && pwd )"
cd ${CATKIN_DIR}
echo "[INFO] Start to catkin_make in the target workspace."
catkin_make