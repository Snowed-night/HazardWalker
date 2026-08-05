#!/usr/bin/env bash
set -eo pipefail
C='simenv_ros1_hazard_platform'
docker ps --format '{{.Names}}' | grep -q "$C" || { echo "ERROR: $C not running"; exit 1; }
cmd="${1:-}"; [ -z "$cmd" ] && { echo 'Usage: hw_service_call.sh door/elevator ...'; exit 1; }
case "$cmd" in
  door)
    echo "[hw/set_door_state] door_id=${2} open=${3}"
    echo "import rospy; rospy.wait_for_service('/set_door_state'); from building_generator_interfaces.srv import SetDoorState; s=rospy.ServiceProxy('/set_door_state',SetDoorState); r=s('${2}',${3}); print('accepted='+str(r.accepted)+' state='+r.state)" | sed 's/true/True/g; s/false/False/g' | docker exec -i "$C" bash -c 'source /opt/ros/noetic/setup.bash; source .ros1_catkin_ws/devel/setup.bash; python3'
    ;;
  elevator)
    echo "[hw/call_elevator] id=${2} floor=${3} open_doors=${4}"
    echo "import rospy; rospy.wait_for_service('/call_elevator'); from building_generator_interfaces.srv import CallElevator; s=rospy.ServiceProxy('/call_elevator',CallElevator); r=s('${2}',${3},${4}); print('accepted='+str(r.accepted)+' floor='+str(r.current_floor)+' state='+r.state)" | sed 's/true/True/g; s/false/False/g' | docker exec -i "$C" bash -c 'source /opt/ros/noetic/setup.bash; source .ros1_catkin_ws/devel/setup.bash; python3'
    ;;
  *) echo 'Usage: door/elevator'; exit 1 ;;
esac
