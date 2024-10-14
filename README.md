# Multi-Drone-Wind-Turbine-Coverage

## Installation Instructions

### Step 1: Install ROS2 Humble 

Follow the link below. Also note that the installation is Ubuntu (deb packages). 

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 

You could also use the videos in the links below! 

Solution-1 : https://youtu.be/LjaTGLVGtuc?si=M-u2TiThsMbsSOmz  

Solution-2: https://youtu.be/uwCjUAEojQ4?si=imWf8zHO8GZUKSjn  

Solution-2: https://youtu.be/flT3LIIR5qo?si=utb2I2FpNmjya7-D 

Some installations may be extra in the video. If this is not in the link provided for Humble you can ignore it? Also, ignore the "sudo apt install ros-humble-ros-base" installation! 

Then set the environment: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html  

The environment can be set directly with the following command. 

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 

Please verify the environment as below. 

![alt text](https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage/blob/main/readme-resources/environmentsetup.jpg?raw=true)
 

### Step 2:  Nvidia OMINIVERSE AND ISAACSIM Setup 

Follow the instructions in the following video. 

Video: https://youtu.be/qacMTFX-YeQ si=NSHEre3OT3E_m5y6 

1. Initially download https://developer.nvidia.com/omniverse#section-getting-started nvidia ominiverse from this page. 

2. Then do the installation according to the video given above. 

3. Do not forget to go to the location of the omniverse-launcher-linux.AppImage file you downloaded while installing with cd. 

4. Omniverse → Install the omniverse cache first from the Exchange tab 

5. After that. Install Isaacsim compatibility checker. Please install version 4.1.0 from the release. Then run this application and verify that the Nvidia Cuda GPU is active. 

If the GPU is not active, you can use the following commands! Repeat after running these commands to verify! 

sudo ubuntu-drivers list 

sudo ubuntu-drivers install 

sudo ubuntu-drivers install nvidia:535 

Please do not forget to reboot afterwards! 

6. Then download the Isaac sim. Please download version 4.1.0 from release, this step is very important. 

 

![alt text](https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage/blob/main/readme-resources/IsaacSim.jpg?raw=true)
 

### Step 3: MAVROS Setup 

Use the following link  

https://github.com/mavlink/mavros/blob/ros2/mavros/README.md  

Then use the following commands to install mavros on ros2 humble version:  

sudo apt install ros-humble-mavros  

ros2 run mavros install_geographiclib_datasets.sh  

## Alternative:  

Wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh ./install_geographiclib_datasets.sh  

To install this file, access "/opt/ros/humble/lib/mavros$" and execute the following command “sudo ./install_geographic". 

### Step 4: QgroundControl 

Follow the Ubuntu version installation instructions using the link below. 

https://docs.qgroundcontrol.com/master/en/qgc-user guide/getting_started/download_and_install.html  

These instructions are as follows: 

 
![alt text](https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage/blob/main/readme-resources/QGC.jpg?raw=true)

chmod +x ./QGroundControl.AppImage 

./QGroundControl.AppImage  (or double click) 

 

To execute the above commands, please remember to navigate to the location of the QgroundControl.AppImage file. 

 

### Step 5: PX4 Setup 

 

The video in the link given below may help you. 

https://youtu.be/8gKIP0OqHdQ?si=JzzpdVE5NHzkpvz1  

https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example  

Or follow these steps:

1. Firstly, an installation that meets Ubuntu requirements should be made. Then follow the steps in this link https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html. 

git clone https://github.com/PX4/PX4-Autopilot.git –recursive 

 

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh 

 

1. This step will be applied in the pegasus simulator installation. You can ignore it if you want.  

Switch to stable version 1.14.3 and compile the code for software in the loop (SITL) mode: 

###### Go to the PX4 directory 

cd PX4-Autopilot 

 

###### Checkout to the latest stable release 

git checkout v1.14.3 

 

###### Initiate all the submodules. Note this will download modules such as SITL-gazebo which we do not need 

###### but this is the safest way to make sure that the PX4-Autopilot and its submodules are all checked out in 

###### A stable and well-tested release 

git submodule update --init --recursive 

 

###### Compile the code in SITL mode 

make px4_sitl_default none 

 

###### Step 6: Pegasus Simulator Setup 

 

Please watch the video in the link below: 

https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html 

If you get an error in the vim when applying, please load the vim 

sudo apt install vim –y 

If the PX4 installation is already present in the installation, do not worry, continue with the installation. 

 

### Step 7: Setup Multi-Drone-Wind-Turbine-Coverage 

 

git clone https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage.git 

cd Multi-Drone-Wind-Turbine-Coverage 
 

### Step 8: Edit UAVSystemLaunch.sh paths 

Modify the paths in UAVSystemLaunch.sh.


![alt text](https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage/blob/main/readme-resources/UAVSystem.jpg?raw=true)

### Step 9: Move assets to the local Nucleus server 

Use the Nvidia Omniverse interface to create a local Nucleus Server through the Nucleus tab 

Create a directory under your user directory called assets localhost/Users/username/Assets 

Move the contents of the Multi-Drone-Wind-Turbine-Coverage/Assets directory to localhost/Users/username/Assets 

### Step 10: Build the ros2_ws packages 

Here in the installation file, source  

/opt/ros/humble/setup.bash 

cd  Multi-Drone-Wind-Turbine-Coverage

cd ros2_ws 

colcon build --packages-select offboard_control_interfaces 

source install/local_setup.bash 

colcon build

source install/local_setup.bash 

 

If you get an error like the picture below during installation.  

 

![alt text](https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage/blob/main/readme-resources/ros2wserror.jpg?raw=true)
 

Solution:  

sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.22/Modules/ 

 

 

### Step 11: Modify mavros multi_uas.launch 

 
![alt text](https://github.com/ajifoster3/Multi-Drone-Wind-Turbine-Coverage/blob/main/readme-resources/multi_uav.jpg?raw=true)

Modify the contents of /opt/ros/your_distribution/share/mavros/launch/multi_uas.launch to the following: 
 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

If you get a permission error, access the following location.  

cd /opt/ros/humble/share/mavros/launch  

then execute  

/opt/ros/humble/share/mavros/launch 

$ sudo chmod -R 777 multi_uas.launch  

and it will now allow you to paste. 

  

### Step 12: Launch the UAV system 

 

./UAVSystemLaunch.sh 

before executing the above command 

modify it to suit yourself 

 

Cd Multi-Drone-Wind-Turbine-Coverage 

if you reencounter the permission problem when running this command 

Cd Multi-Drone-Wind-Turbine-Coverage 

sudo chmod -R 777 ./UAVSystemLaunch.sh   

Apply 

### Step 13: Observe the IsaacSim window, once the drones have reached their target altitude, run: 

 

./CentralPlanner.sh 





# Multi-Drone-Wind-Turbine-Coverage

## Installation Instructions

### Prerequisites

- **Operating System:** Ubuntu 22.04
- **GPU:** NVIDIA GPU with CUDA support

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
   3. When the top right window displays the following, enter the prepolutated command in the top left window
     ```bash
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     INFO  [commander] Ready for takeoff!
     ```
   4. Switch tmux window with ctrl+b -> down arrow
   5. When the top right window displays the following, enter the prepolutated command in the bottom left window
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
   
