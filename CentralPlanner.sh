source /opt/ros/humble/setup.bash
source ~/Documents/Github/Multi-Drone-Wind-Turbine-Coverage/ros2_ws/install/local_setup.bash

ros2 run central_ground_coverage_controller CentralCoverageController 5 GreedyIterative /home/ajifoster3/Downloads/all_geoposes_wind_turbine.json

