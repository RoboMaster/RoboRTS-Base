#!/bin/bash
CATKIN_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." >/dev/null 2>&1 && pwd )"
echo "[INFO] Start to launch the roborts_base_node."
/bin/bash -c "source ${CATKIN_DIR}/devel/setup.bash && roslaunch roborts_base base.launch"
