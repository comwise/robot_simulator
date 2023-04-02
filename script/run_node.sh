#!/usr/bin/env bash

shutdown() {
  # Get our process group id
  PGID=$(ps -o pgid= $$ | grep -o [0-9]*)
  # Kill it in a new new process group
  kill_node
  setsid kill -2 -- -${PGID}
  exit 0
}

kill_node() {
  ps -ef | grep robot_sim | grep -v grep | cut -c 9-15 | xargs kill -9
}

trap 'shutdown' SIGINT SIGTERM

source /opt/ros/kinetic/setup.bash --extend|| { echo `date` ": source ros bash failed"; exit 1; }
source /opt/cotek/robot_sim/devel/setup.bash --extend  || { echo `date` ": source robot_sim bash failed"; exit 2; }
/opt/robot/robot_sim/bin/sim_node  || { echo `date` ": run robot_sim failed"; exit 3; }
wait
exit 0
