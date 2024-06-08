#!/bin/bash

# Name of the tmux session
SESSION="DevSession"

cleanup() {
  tmux kill-session -t $SESSION
}

# Trap EXIT signal to cleanup tmux session
trap cleanup EXIT

# Start a new tmux session and detach from it
tmux new-session -d -s $SESSION

# Split the window into four panes
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v


# Pane 1: source ROS setup script and type ros2 launch mavros multi_uas.launch without running
tmux send-keys -t $SESSION:0.0 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:0.0 'ros2 launch mavros multi_uas.launch' 

# Pane 2: cd to ~/Documents/Code, source ROS setup scripts, and type ./multi_launch_drones.sh without running
tmux send-keys -t $SESSION:0.1 'cd ~/Documents' C-m
tmux send-keys -t $SESSION:0.1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:0.1 'source ~/Documents/GitHub/Multi-Drone-Wind-Turbine-Coverage/ros2_ws/install/local_setup.bash' C-m
tmux send-keys -t $SESSION:0.1 './multi_launch_drones.sh' 

# Pane 3: cd to ~/Applications/PegasusSimulator/examples and run ISAACSIM_PYTHON 2_px4_multi_vehicles.py
tmux send-keys -t $SESSION:0.2 'cd ~/Desktop' C-m
tmux send-keys -t $SESSION:0.2 'ISAACSIM_PYTHON Pegasus_sim_launch.py' C-m

# Pane 4: cd to ~/Applications and run ./QGroundControl.AppImage
tmux send-keys -t $SESSION:0.3 'cd ~/Applications' C-m
tmux send-keys -t $SESSION:0.3 './QGroundControl.AppImage' C-m

# Attach to the session
tmux attach-session -t $SESSION

