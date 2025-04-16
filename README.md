# abu2025_ros2 package

abu2025_ros2 is a gameplay system and base system launch file for ABU Robocon 2025.  
By Robot Club Engineering KMITL team RoBoRho.

# ROS Dependencies
- ros-humble-robot-state-publisher (sudo apt install)
- ros-humble-xacro (sudo apt install)
- entire iRob_bot_ros2 package set (git clone and colcon build)
- ust_05ln_ros2 (git clone and colcon buid)
- sllidar_ros2 (git clone and colcon build)
- cartographer_ros (git clone and colcon build on Jetson / sudo apt install on Raspberry Pi)
- rViz (optional, recommended on PC but not on Pi)

# Python Dependencies
- transforms3d (pip3 install transforms3d)

# Build instruction
0. Install the mentioned ROS dependencies and Python dependencies, put some package that requires ```git clone``` in the save workspace as the ```abu2025_ros2``` package
1. create your ROS2 workspace. For example ```abu2025_ws```
2. cd in to your workspace and create folder ```src```
3. cd in to ```src``` and ```git clone https://github.com/E12-CO/abu2025_ros2.git```
4. cd .. back outside the ```src``` folder
5. before builing the ```abu2025_ros2``` **make sure to install all dependencies!**
6. run ```colcon build --symlink-install --packages-select abu2025_ros2```
7. after colcon completed, don't forget to source the workspace. In the real robot, this will be automatically done via .bashrc 

# Simulation commands

To start the Gazebo Robot simulation (and iRob maneuv3r)
```
ros2 launch abu2025_ros2 launch_sim.launch.py
```
and for the SLAM simulation 
```
ros2 launch abu2025_ros2 carto_localization.launch.py
```

# Real robot base system

To **START** base system, this includes iRob hardware interface, Joy controller interface and LiDARs.  

On R1 (Front LiDAR Hokuyo 8 meters, rear LiDAR RPLiar C1) run
```
ros2 launch abu2025_ros2 r1_base.launch.py
```

On R2 (Front and rear LiDAR are Hokuyo 8 meters) run
```
ros2 launch abu2025_ros2 r2_base.launch.py
```

# Create game field map with real R1 robot
To **MAP** the game field with Cartographer (**available for R1 only**) 
1. Make sure that the center of the robot is away from field edge around 1 meters
2. Make sure that the robot base system is running.
3. run the following command to start Cartographer 
```
ros2 launch abu2025_ros2 r1_mapping.launch.py
```
Walk the robot around the edge of the game field and back to the start point.  
To save map, run this in other terminal while the cartographer map is still running
```
ros2 service call /r1/write_state cartographer_ros_msgs/srv/WriteState "filename: abu2025_map.
pbstream"
```
Map should be saved in home directory. If not, looking for the map file inside the workspace folder.
Copy this map file and place in the abu package folder in that same workspace
```
abu2025_ros2/maps
```
Then run step 5. in the Build instruction

# Node-red web UI
**Node-red WebUI will be hosted on the Jetson Nano!** To install the Node-red, follow the steps
1. Install node-red using [this guide](https://nodered.org/docs/getting-started/raspberrypi). Start node-red once and stop it
2. copy ```flows.json``` and ```settings.js``` in the ```abu2025_ros2/nodered``` package folder to ```.node-red``` folder at your home folder
3. at your home folder, create a folder named ```pics``` and copy all images from ```abu2025_ros2/nodered``` to here
4. start the node-red using 
```
sudo systemctl enable nodered.service
```
and 
```
sudo systemctl start nodered.service
```

To edit the node-red. Enter the website of Jetson nano IP address with port 1880
```
jetson.ip.addr.here:1880
```
Username and password are
```
user: ajb 
password: ajb2024
```

To access the node-red dashboard on PC or tablet, visit the website
```
jetson.ip.addr.here:1880/ui
```