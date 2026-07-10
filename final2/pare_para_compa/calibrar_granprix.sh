#!/bin/bash
ros2 param set /maze_solver active false
ros2 param set /maze_solver v_forward 0.07
ros2 param set /maze_solver turn_speed 0.16
ros2 param set /maze_solver kp_wall 0.15
ros2 param set /maze_solver wall_target 0.25
ros2 param set /maze_solver front_stop 0.40
ros2 param set /maze_solver side_open 1.50
ros2 param set /maze_solver intersection_cooldown_s 4.0
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
ros2 param set /pare_detector min_area 20000
ros2 param set /pare_detector max_area 300000
ros2 param set /pare_detector min_ratio 0.15
ros2 param set /pare_detector max_ratio 6.0
ros2 param set /pare_detector min_area 8000
ros2 param set /pare_detector max_area 300000
ros2 param set /pare_detector min_ratio 0.10
ros2 param set /pare_detector max_ratio 8.0
