#!/bin/bash

# Define the commands in an array
commands=(
  "ros2 run offboard_control takeoff 1 centralised"
  "ros2 run offboard_control takeoff 2 centralised"
  "ros2 run offboard_control takeoff 3 centralised"
  "ros2 run offboard_control takeoff 4 centralised"
  "ros2 run offboard_control takeoff 5 centralised"
)

# Export ROS_DOMAIN_ID if needed (uncomment if required)
# export ROS_DOMAIN_ID=1

# Run the commands in parallel
printf "%s\n" "${commands[@]}" | xargs -I {} -P 0 bash -c "{}"
