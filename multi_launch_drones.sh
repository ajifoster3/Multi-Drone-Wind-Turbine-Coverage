#!/bin/bash

# Start the first ROS 2 node and run it in the background
ros2 run offboard_control takeoff 1 centralised &
ros2 run offboard_control takeoff 2 centralised &
ros2 run offboard_control takeoff 3 centralised &
ros2 run offboard_control takeoff 4 centralised &
ros2 run offboard_control takeoff 5 centralised 

