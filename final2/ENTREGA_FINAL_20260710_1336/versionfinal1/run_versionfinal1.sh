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
python3 /root/VERSIONES_CASA/versionfinal1/rescate_chaski_versionfinal1.py --ros-args -p mode:=corridor -p hand:=left -p v:=0.035 -p v_slow:=0.022 -p w:=0.14 -p max_wz:=0.08 -p kp:=0.12 -p wall_target:=0.30 -p front_stop:=0.46 -p front_danger:=0.25 -p side_danger:=0.12 -p turn_time:=0.72 -p back_time:=0.30 -p start_straight_s:=3.0 -p pare_stop_s:=3.0 -p force_first_left:=true -p first_left_release:=0.64
