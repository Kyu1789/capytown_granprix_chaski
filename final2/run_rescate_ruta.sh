#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=20
pkill -9 -f joy_ctrl || true
pkill -9 -f joy_node || true
pkill -9 -f yahboom_joy || true
pkill -9 -f maze_solver || true
pkill -9 -f maze_safe || true
pkill -9 -f chaski_solver || true
pkill -9 -f chaski_corridor || true
pkill -9 -f laser_Avoidance || true
pkill -9 -f rescate_chaski || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
python3 /root/rescate_chaski.py --ros-args -p mode:=route -p route:="S4.0,L0.72,S2.5,R0.72,S2.5,L0.72,S3.0,R0.72,S3.0" -p v:=0.035 -p v_slow:=0.022 -p w:=0.14 -p front_stop:=0.44 -p front_danger:=0.24 -p start_straight_s:=0.0 -p pare_stop_s:=3.0
