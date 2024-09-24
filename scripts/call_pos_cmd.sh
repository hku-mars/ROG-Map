#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 3 ]; then
  echo "Usage: ./control_drone.sh <x> <y> <z>"
  exit 1
fi

# Assign input arguments to variables
X_POS=$1
Y_POS=$2
Z_POS=$3

# Publish the PositionCommand message to control the drone
rostopic pub /planning/pos_cmd quadrotor_msgs/PositionCommand "header:
  seq: 1
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'base_link'
position:
  x: $X_POS
  y: $Y_POS
  z: $Z_POS
velocity:
  x: 0.0
  y: 0.0
  z: 0.0
acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
jerk:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
attitude:
  x: 0.0
  y: 0.0
  z: 0.0
thrust:
  x: 0.0
  y: 0.0
  z: 0.0
yaw: 0.0
yaw_dot: 0.0
vel_norm: 0.0
acc_norm: 0.0
kx: [0.0, 0.0, 0.0]
kv: [0.0, 0.0, 0.0]
trajectory_id: 1
trajectory_flag: 1" -1
