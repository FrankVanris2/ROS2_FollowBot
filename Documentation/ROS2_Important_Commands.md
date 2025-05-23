# Important commands/information for ROS2 (Linux)

<h2>Starting from personal PC</h2>

When starting the ROS2 you will need to ssh into the raspberry pi from a personal pc via this command
```
#ssh ubuntu@CURRENT_RASPBERRYPI_IP_ADDRESS

#in my case
ssh ubuntu@10.0.0.25
```
when enabling X11 do this command:
```
ssh -X ubuntu@10.0.0.25 (with your current raspberry pi ip address of course)
```

Doing this will then give you a prompt for the password to the pi, the password is: robotics101
<h2>Sourcing the setup files: </h2>  

```
source /opt/ros/jazzy/setup.bash
```

**(optional/necessary)** adding to sheet startup script: (Note: if need to remove, locate startup script file)
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

<h2> Check environment variables </h2>

```
printenv | grep -i ROS
```

When doing this you should see the following:

```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=jazzy
```


The `ROS_DOMAIN_ID` variable:

```
export ROS_DOMAIN_ID=<your_domain_id>
```
When it comes to the ID its specified to keep things in the range between 0 and 101 inclusivly. 

**(optional/necessary)** Adding to sheet to startup script: (Note: if need to remove, locate startup script file)
```
echo " export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```

The `ROS_LOCALHOST_ONLY` variable:  
```
export ROS_LOCALHOST_ONLY=1
```
This variable allows you to limit the ROS 2 communication to localhost only. This means your ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network.

**(optional/necessary)** Adding to sheet to startup script: (Note: if need to remove, locate startup script file)

```
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

<h3>Note: </h3>

When needing to remove a given command within .bashrc you will need to go into that file via these commands:

```
#for locating the file
ls -a

#for entering into the file
vi .bashrc
```

Important commands when editing a file:

```
i: for inserting

esc: for getting out of inserting

Shift ZZ: for saving the file and quickly exiting out of the file

:q! -> for exiting the file without saving
```

Important command when exiting an environment:
```
To exit an environment properly ALWAYS do ctrl + C
```

<h1> Submodule Guide </h1>

Our project relies on [LDROBOT LiDAR LD14P](https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2/tree/d70802ac5d46e4e02c8318b2769f508f0f86172e) hardware driver provided by ShenZhen LDROBOT Co., LTD , Sensor team. This is embedded as a submodule via the following commands:
```
cd ~/fb_ws/ROS2_FollowBot/FollowBotAROS2/src
git submodule add https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git ldlidar_sl_ros2
git add .gitmodules src/ldlidar_sl_ros2 # stage the submodule config and path

git commit -m "add ldlidar_sl_ros2 submodule"
git push
```

In order to update the submodule 
```
# updating submodules (if the main project’s submodule pointer changes):
git pull  # pull main project changes
git submodule update --recursive

# pulling latest submodule changes (to update the submodule itself):
cd src/ldlidar_sl_ros2
git pull origin main  # or your branch
cd ../..
git add src/ldlidar_sl_ros2
git commit -m "update submodule"
```


<h1> Windows GUI for Running the Pi </h1>
VcXsrv is our chosen X server that allows you to run any graphical Linux application on Windows, including more complex simulations. This is especially useful for running RViz, which visualizes sensor data.

* Download and install VcXsrv from [SourceForge](https://sourceforge.net/projects/vcxsrv/).

To enable graphical applications from the Raspberry Pi or remote Ubuntu machine to display on Windows, update your .bashrc file:

```
vi .bashrc
```
Add the following line at the end of the file, replacing ${YOUR_IP_ADDRESS_ON_CURRENT_MACHINE} with your Windows machine's IP address:
```
export DISPLAY=${YOUR_IP_ADDRESS_ON_CURRENT_MACHINE}:0
```

Then open XLaunch; make sure to set `Clipboard`, `Primary Selection` and `Disable access control` and unset `Native opengl`. Once VcXsrv is running, execute the following command in Ubuntu to verify Rviz2 is displayed on your Windows computer:

```
ros2 run rviz2 rviz2
```

<h1> Gazebo Simulation Environment </h1>
Gazebo Harmonic is the most compatible version for ROS2 Jazzy and is required for running simulations. It can be installed with the following commands:

```
sudo apt-get install curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

Check to see if Gazebo works with a basic command: `gz sim shapes.sdf`. If not, this may work:
```
sudo apt-get install ros-jazzy-gz-tools-vendor
sudo apt-get install ros-jazzy-gz-sim-vendor
. /opt/ros/jazzy/setup.bash
```

<h1> slam-toolbox Set-Up</h1>
Install with:

```
sudo apt-get update
sudo apt install ros-jazzy-slam-toolbox
```

We plan to use online asynchronous SLAM, which allows real-time mapping while the robot is moving. To start, copy the default configuration file into your development directory:

```
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml {my}/{dev}/{directory}

# example:
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml Projects/ROS2_FollowBot/FollowBotAROS2/src/mapping/config/
```

Once the parameters are adjusted to fit our setup, run `slam-toolbox` using the following command:

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./Projects/ROS2_FollowBot/FollowBotAROS2/src/mapping/config/mapper_params_online_async.yaml
```

(With a LiDAR sensor publishing to `/scan`) In a seperate terminal, run Rviz to visualize the mapping process:

```
ros2 run rviz2 rviz2
```

In RViz, you should now see available topics such as: 
* `/map` - The generated occupancy grid.
* `/scan` - Incoming LiDAR or depth camera data.
* `/odom` - Odometry data from the bot’s sensors.

<h1> robot_localization set-up </h1>
Install with:

```
sudo apt-get update
sudo apt-get install ros-jazzy-robot-localization
```

Key nodes:
* `ekf_filter_node_odom` to provide local odometry via fusing IMU and encoder sensor data
* `ekf_filter_node_map` to provide global odometry via fusing GPS data
* and [navsat_transform_node](http://docs.ros.org/en/jade/api/robot_localization/html/navsat_transform_node.html) produces an odometry output with the position of the GPS in the map frame

Key topics:
| Topic                   | Message Type                   | Purpose                                      |
|-------------------------|--------------------------------|----------------------------------------------|
| `/wheel_odom`           | `nav_msgs/Odometry`            | Input wheel odometry data (remapped from `/odom`). |
| `/imu/data`             | `sensor_msgs/Imu`              | Input IMU orientation and angular velocity. |
| `/gps/fix`              | `sensor_msgs/NavSatFix`        | Input GPS global position data.             |
| `/odometry/filtered`    | `nav_msgs/Odometry`            | Output fused odometry estimate (EKF result).|

`./mapping/config/dual_ekf_navsat_params.yaml` will show all remappings of topics and what the package now expects from our configuration. Hopefully this graph helps visualize this package at work:

![image](https://github.com/user-attachments/assets/51308c7b-d7ec-4a92-a3f2-73d32f0c1200)



<h1> nav2 set-up </h1>
Install with:

```
sudo apt-get update
sudo apt-get install ros-jazzy-navigation2 ros-humble-nav2-bringup
```
Key nodes:
* `controller_server`: generates velocity commands (/cmd_vel) to follow the path
* `planner_server`: plans paths to goals (/goal_pose)
* `bt_navigator`: executes navigation behavior trees
* `amcl`: localization using Adaptive Monte Carlo Localization

Key topics:
| Topic               | Message Type                                | Purpose                                      |
|---------------------|---------------------------------------------|----------------------------------------------|
| `/cmd_vel`          | `geometry_msgs/Twist`                       | Output velocity commands to the robot base.  |
| `/goal_pose`        | `geometry_msgs/PoseStamped`                 | Input goal pose for navigation.              |
| `/amcl_pose`        | `geometry_msgs/PoseWithCovarianceStamped`   | Estimated robot pose from AMCL.              |
| `/map`              | `nav_msgs/OccupancyGrid`                    | Static map for localization and planning.    |

<h2>rqt_graph</h2>
rqt graph will display the given information about the nodes and the topics that are currently being played with via this command for ROS2:

```
ros2 run rqt_graph rqt_graph
```

![image](./Images/rqt_graph.png)

the rectangles are the topics, and the ovals are the nodes.
nodes will send messages through services which will be passed down through topics in which those topics will send those messages to other nodes.

rqt graph is a helpful tool to see how your nodes interact with one another. This will be beneficial when you visually need to see what is happening with your robotic system.


<h2>Services</h2>
Nodes can communicate using services in ROS 2. Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.

You generally don’t want to use a service for continuous calls; topics or even actions would be better suited.<br>

Important service commands:

```
#important commands:

ros2 service list

ros2 service find <type_name> 

EXAMPLE: ros2 service find std_srvs/srv/Empty

ros2 interface show <type_name>.srv

EXAMPLE: ros2 interface show std_srvs/srv/Empty.srv

ros2 service call <service_name> <service_type> <arguments>

EXAMPLE: ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

NOTE: when passing in arguments for most of these services, topics, etc. You need to always write it in YAML format. Similar to JSON format or HTML format.

<h2> Important Notes: </h2>

**Parameters**: is a configuration value of a node, think of them as node settings<br>
* can store parameters as integers, floats, booleans, strings, and lists.
* each node maintains it's own parameters.

```
#to check parameter lists do this:
ros2 param list

#to determine a parameter's type, you can use: 
ros2 param get <node_name> <parameter_name>

EXAMPLE:
ros2 param get /turtlesim background_g

#To change a parameters value at runtime, use the command:
ros2 param set <node_name> <parameter_name> <value>

EXAMPLE:
ros2 param set /turtlesim backgroun_r 150

#These values will only show up in your current session, but you can save them so they can show up in every session via this command:
ros2 param dump <node_name>

EXAMPLE:
ros2 param dump /turtlesim

#You can load parameters from afile to a currently running node using the command:
ros2 param load <node_name> <parameter_file>

EXAMPLE:
ros2 param load /turtlesim ./turtlesim.yaml

#To startthesame node using your saved parameter values, use:
ros2 run <package_name> executable_name> --ros-args --params-file <file_name>

EXAMPLE:
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
```

<h2> Actions </h2> 
Actions are quite easy to learn and I believe it's best to read the whole tutorial of it to understand it fully:

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html


<h2>RQT Console</h2>

This will be very useful and handy to know whether there is an issue with our Followbot:

```
#Command to open the console:
ros2 run rqt_console rqt_console
```
![image](./Images/rqt_console.png)

The loggers level of severity is ordered in this manner:
There is no exact standard for what each level indicates, but it’s safe to assume that:

* `Fatal` messages indicate the system is going to terminate to try to protect itself from detriment.

* `Error` messages indicate significant issues that won’t necessarily damage the system, but are preventing it from functioning properly.

* `Warn` messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don’t harm functionality outright.

* `Info` messages indicate event and status updates that serve as a visual verification that the system is running as expected.

* `Debug` messages detail the entire step-by-step process of the system execution.
<br>
<br>

`rqt_console` can be very helpful if you need to closely examine the log messages from your system. You might want to examine log messages for any number of reasons, usually to find out where something went wrong and the series of events leading up to that.

<h2>Recording and playing back data</h2>
To record the data published to a topic use the command syntax:

```
ros2 bag record <topic_name>

#EXAMPLE:
ros2 bag record /turtle1/cmd_vel
```

When you are done recording do `ctrl + C`

Recording multiple topics:

```
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

`-o` allows you to choose a unique name for the bag
`subset` is the name of the bag you are choosing.

You can record data passed on topics in your ROS 2 system using the ros2 bag command. Whether youre sharing your work with others or introspecting your own experiments, its a great tool to know about.

There are more commands and information found on the wiki and the documentation page.

<h2> Colcon IMPORTANT COMMAND </h2>

When building Colcon on a raspberry pi with 1GB of RAM it is best advised
to not use this command:

```
#DO NOT USE THIS COMMAND
colcon build --symlink-install
```
Doing this will only crash your build and will cause more trouble in the end.
To fix this you must run the following command:
```
#DO THIS
export MAKEFLAGS="-j 1"
colcon build --executor sequential
```
This will use 1 core of your cpu to run the build which will put less load on the RAM of the 
Raspberry pi.

#Creating a Workspace

* first be sure to source the ros version that you are using in the bash file.

* Then make a new directory for your workspace

```
mkdir -p ~/ros2_ws_name/src
cd ~/ros2_ws/src
```

* When it comes to the workspace that is where you will be creating your own packages. Sometimes you'll download repo's

* Next in order to use your package or use another package you'll need to resolve some dependencies. It's always good practice to do this in order to see if there are any dependencies you'll need to run the package. Do the following to check, just make sure your not in the src:

```
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone

cd ..
rosdep install -i --from-path src --rosdistro jazzy -y
```

* If you already have all your dependencies, the console will return:
`#All required rosdeps installed successfully`

## Creating a workspace and packages

To create a workspace be sure it has a source. Do this command:

```bash
mkdir -p ~/your_workspace/src
cd ~/your_workspace/src
```

When in your own workspace, creating packages will be essential. Especially when making publisher-subscriber architecture.
Here is a command you can do in Cmake AND Python:

`CMake`
```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
```

`Python`
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```

Packages in workspaces should look similar to this:

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

## Building Packages

When it comes to building packages you need to ensure that all your dependencies are taken care of.
Be sure to do this command, it will ensure you if you need to download other dependencies or not:

```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

To Build a package make sure you are in your root directory. After you migrate to your
root directory input this:

```bash
 colcon build --packages-select cpp_pubsub
 ```
