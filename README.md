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

### Step 2: Install PX4 and mavros

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

4. **Build the ros2_ws packages**
   ```bash
   source /opt/ros/.../install/setup.bash
   cd ros2_ws
   colcon build --packages-select offboard_control_interfaces
   source install/local_setup.bash
   colcon build
   source install/local_setup.bash
   ```
5. **Modify mavros multi_uas.launch**
   - Modify the contents of /opt/ros/your_distribution/share/mavros/launch/multi_uas.launch to the following:
   ```launch
   <launch>
    <!-- px4_0 -->
    <group>
        <arg name="id" default="0" />
        <arg name="fcu_url" default="udp://:14540@127.0.0.1:14580" />
        <include file="$(find-pkg-share mavros)/launch/px4.launch">
            <arg name="tgt_system" value="$(eval '1 + int(\'$(var id)\') ')" />
            <arg name="namespace" value="$(eval ' \'mavros/uas_\' + \'$(var tgt_system)\' ')" />
        </include>
    </group>
    <!-- px4_1 -->
    <group>
        <arg name="id" default="1" />
        <arg name="fcu_url" default="udp://:14541@127.0.0.1:14581" />
        <include file="$(find-pkg-share mavros)/launch/px4.launch">
            <arg name="tgt_system" value="$(eval '1 + int(\'$(var id)\') ')" />
            <arg name="namespace" value="$(eval ' \'mavros/uas_\' + \'$(var tgt_system)\' ')" />
        </include>
    </group>
    <!-- px4_2 -->
    <group>
        <arg name="id" default="2" />
        <arg name="fcu_url" default="udp://:14542@127.0.0.1:14582" />
        <include file="$(find-pkg-share mavros)/launch/px4.launch">
            <arg name="tgt_system" value="$(eval '1 + int(\'$(var id)\') ')" />
            <arg name="namespace" value="$(eval ' \'mavros/uas_\' + \'$(var tgt_system)\' ')" />
        </include>
    </group>
    <!-- px4_3 -->
    <group>
        <arg name="id" default="3" />
        <arg name="fcu_url" default="udp://:14543@127.0.0.1:14583" />
        <include file="$(find-pkg-share mavros)/launch/px4.launch">
            <arg name="tgt_system" value="$(eval '1 + int(\'$(var id)\') ')" />
            <arg name="namespace" value="$(eval ' \'mavros/uas_\' + \'$(var tgt_system)\' ')" />
        </include>
    </group>
    <!-- px4_4 -->
    <group>
        <arg name="id" default="4" />
        <arg name="fcu_url" default="udp://:14544@127.0.0.1:14584" />
        <include file="$(find-pkg-share mavros)/launch/px4.launch">
            <arg name="tgt_system" value="$(eval '1 + int(\'$(var id)\') ')" />
            <arg name="namespace" value="$(eval ' \'mavros/uas_\' + \'$(var tgt_system)\' ')" />
        </include>
    </group>
    </launch>
   ```  
   
6. **Launch UAV system**
   ```bash
   ./UAVSystemLaunch.sh
   ```
   1. An IsaacSim and Q-groundcontrol window will launch
   2. Switch tmux window with ctrl+b -> up arrow
   3. When the top right window displays the following enter the prepolutated command in the top left window
     ```bash
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     ```
   4. Switch tmux window with ctrl+b -> down arrow
   5. When the top right window displays the following enter the prepolutated command in the bottom left window
    ```bash
    INFO  [mavlink] partner IP: 127.0.0.1
    INFO  [mavlink] partner IP: 127.0.0.1
    INFO  [mavlink] partner IP: 127.0.0.1
    INFO  [mavlink] partner IP: 127.0.0.1
    INFO  [mavlink] partner IP: 127.0.0.1
    ```
   6. You should now see (If not please repeat the previous steps)
   ```bash
   INFO  [commander] Takeoff detected
   INFO  [commander] Takeoff detected
   INFO  [commander] Takeoff detected
   INFO  [commander] Takeoff detected
   INFO  [commander] Takeoff detected
   ```
   7. Observe the IsaacSim window, once the drones have reached their target altitude, run:
   ```bash
   ./CentralPlanner.sh
   ```
   
