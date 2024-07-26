# Multi-Drone-Wind-Turbine-Coverage

## Installation Instructions

### Prerequisites

- **Operating System:** Ubuntu 18.04/20.04 or Windows 10
- **GPU:** NVIDIA GPU with CUDA support
- **Software Dependencies:** Docker, NVIDIA Docker, Python 3.7+

### Step 1: Install IsaacSim

1. **Update your system:**
    ```bash
    sudo apt-get update
    sudo apt-get upgrade
    ```
    
2. **Download and Install IsaacSim:**
   - [IsaacSim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)

### Step 3: Install PX4 and mavros

- Install [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- Install [mavros](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md)
     
### Step 3: Download, Install and setup Pegasus Simulator

1. **Clone the Pegasus Simulator repository:**
    - [Pegasus Simulator Install](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)
    - Modify the global_coordinates config at PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml to:
        ```bash
        global_coordinates:
        altitude: 230
        latitude: 55.70972
        longitude: -4.33519
        ```
    
### Step 4: Set up Multi-Drone-Wind-Turbine-Coverage

1. **Clone the repository:**
    ```bash
    git clone https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage.git
    cd Multi-Drone-Wind-Turbine-Coverage
    ```
2. **Edit UAVSystemLaunch.sh paths**
   - Modify the paths within the UAVSystemLaunch.sh file in accordance with your directory setup

3. **Move assets to local Nucleus server**
   - Use the Nvidia Omniverse interface to create a a local Nucleus Server throught the Nucleus tab
   - Create a directory under your user directory called assets localhost/Users/username/Assets
   - Move the contents of the Multi-Drone-Wind-Turbine-Coverage/Assets directory to localhost/Users/username/Assets
