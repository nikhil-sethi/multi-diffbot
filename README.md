# BullProof Techologies
<p align="center">
<img src="https://drive.google.com/uc?id=1R4BYatSzIrdLh8kk5JZJNoghUWQ8TwpW" alt="Team Logo" width="30%" height="30%" title="Team Logo" > 
</p>

> Team Logo of BullProof Technologies

This repostory contains the Multi-Disciplinary Project of 2022/2023 of Group 8, developed for Lely. This ReadMe file contains instructions to install and set up the robot, information about the packages contained in the repository and a brief overview of the node structure used by the robot.


## Table of Contents
- [Installation, Setup and Robot Shutdown](#installation-setup-and-robot-shutdown)
    - [Cloning repostory](#cloning-repository)
    - [Building the workspace](#building-the-workspace)
    - [Connecting to the robot](#connecting-to-the-robot)
    - [Starting the robot](#starting-the-robot)
    - [Shutting down the robot](#shutting-down-the-robot)
- [Packages](#packages)
    - [bullproof_bringup](#bullproof_bringup)
    - [bullproof_control](#bullproof_control)
    - [bullproof_hri](#bullproof_hri)
    - [bullproof_nav](#bullproof_nav)
    - [bullproof_perception](#bullproof_perception)
    - [mirte-ros-packages](#mirte-ros-packages)
- [Software Architecture](#software-architecture)
    - [camera_top_view](#camera_top_view)
    - [map_server](#map_server)
    - [map_update](#map_update)
    - [farmer_planner](#farmer_planner)
    - [robot_planner](#robot_planner)
    - [move_base](#move_base)

# Installation, Setup and Robot Shutdown
## Cloning repository
To clone this repository, please open a terminal in the folder where you would like to create the repository and run the following command:

``` bash
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-8/bullproof-tech.
```
The repository should now be cloned on your machine.
## Building the workspace
To build the repository, open the `bullproof-tech` folder (=location of the repository) in a terminal and run the following commands:

``` bash
cd bullproof_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-noetic-teb-local-planner
catkin_make
```
> **NOTE:** Do not use `rosdep2 install`. Make sure you are using rosdep for ROS Noetic! Any other version may result in unwanted consequences. The manual installation of `ros-noetic-teb-local-planner` may also be required if rosdep cannot find this package.

This will update any submodules, install dependencies and then build the workspace for ROS within the workspace folder `bullproof_ws`.

## Connecting to the robot
Firstly, one must connect to the WiFi network that the Mirte robot creates. Look for and connect to the following WiFi network:
* Network: mirte_bullproof
* Password: mirte_mirte

Once connected, one can connect to the `$mirte` shell via ssh by running the following commands:

```bash
$ ssh mirte@192.168.42.1
```

It will then ask for a password, this password is "bullproof". Once connected, you now have access to the `$mirte` shell. This is required to shut the robot down later.

## Connecting the robot to the camera network
> Note: Anything labeled (first time) only has to be done when using a fresh Mirte. The Mirte used by this group is already configured.

Within the arena, there is an overhead camera located that is used by this project. This camera is in essence another Mirte robot, but only serves to publish to a camera topic. In order to connect to this camera, follow these instructions:

### Connecting to the camera network
First, connect to the following WiFi network:
* Network: Lely Camera
* Password: Camera01

Then, connect to the `$mirte` shell of the camera (caled `$camera` in this ReadMe for clarity).

```bash
$ ssh mirte@192.168.20.198
```

It will ask you for confirmation, and then a password, which is "dev_dev".

Now open the `etc/hosts file on the Camera Mirte:

```bash
$camera sudo nano /etc/hosts
```

Add your PC's IP and IP name (on the Lely Camera Network) to this file. If you do not know your IP address, run `$ ifconfig` or `$ ip a` to check.

Now add the Mirte to your _own_ `etc/hosts` file:

```bash
$ sudo nano /etc/hosts
```
And add a line with the following details:
* IP-address: 192.168.20.198
* IP-name: Mirte-2917C3

To test whether the camera is working, run the following commands:

```bash
$ export ROS_MASTER_URI=http://192.168.20.198:11311
$ export ROS_IP=<your_wlan_ip>
```

Add your PC's IP and IP name (on the Lely Camera Network) to this file. If you do not know your IP address, run `$ ifconfig` or `$ ip a` to check. If you can see the topic `/webcam/image_raw/compressed` or any similar topics, it is connected correctly.

### Connecting Mirte to camera network (first time)
Now you must connect the Mirte robot to the camera network. To do so, re-open the `$mirte` shell from [Connecting to the robot](#connecting-to-the-robot), and run the following command:

```bash
$mirte nohup `$(sleep 10; sudo nmcli d wifi connect "Lely Camera"
password "Camera01")` & sudo nmcli d disconnect wlan0
```
This will disconnect the "mirte_bullproof" network and connect the Mirte robot to the "Lely Camera" network. Re-connect your PC to the "Lely Camera" network, and run the following command to connect to the `$mirte` shell:

```bash
ssh mirte@192.168.20.104
```
Again, the password is "bullproof". Now you are once again connected to the `$mirte` shell, but it is now located on the "Lely Camera" network.

### Setting the Camera as ROS Master (first time)
In order to run ROS nodes on both the Camera and Mirte, one must be set as the ROS master. This will be done on the Camera. In order to make the Camera the ROS Master, connect to the `$mirte` shell and run the following commands.

> Note: This only has to be done once, when you are starting from a fresh Mirte robot. For the current Mirte robot we are using, this does not have to be done again.

```bash
$mirte sudo nano mirte_ws/devel/setup.bash
```

And add the following lines to the end of the file (make sure you are connected to the "Lely Camera" network):

```
ip4=$(/sbin/ip -o -4 addr list wlan0 | awk ’{print $4}’ | cut -d/ -f1)
export ROS_IP=$ip4
if [[ $ip4 == 192.168.20.* ]] ; then
export ROS_MASTER_URI=http://192.168.20.198:11311
fi
```

This will set the ROS IP variable to the wifi ip address and the ROS MASTER URI to the camera Mirte if it is connected to the ”Lely Camera” wifi network.

Now open the following file:
```bash
$mite sudo nano mirte_ws/src/mirte-ros-packages/mirte_bringup/launch/minimal.launch
```

And remove any launch files for the camera nodes. Now close and save the file.

### Restart the ROS service on the Mirte robot (in case of error)
Depending on the start-up, the ROS service on the Mirte robot may not start correctly. Examples of this are when you edited the `devel/setup.bash` as described in the last step. In order to fix any issues, restart the ROS service on the Mirte as follows:

```bash
$ rosnode cleanup
$mirte sudo systemctl stop mirte-ros
$mirte sudo systemctl start mirte-ros
```
## Starting the robot
Before starting the robot or running any packages, you must run the following in the user shell `$` (not `$mirte`):

```bash
$ export ROS_MASTER_URI=http://192.168.20.198:11311
$ export ROS_IP=<your_wlan_ip>
```

Add your PC's IP and IP name (on the Lely Camera Network) to this file. If you do not know your IP address, run `$ ifconfig` or `$ ip a` to check.

Once connected, you can now execute any scripts or commands to the ROS master on the Mirte robot. This must be re-done for every new shell instance. You do not need to run `$roscore` locally.

In order to start-up the robot properly, [bullproof_bringup](#bullproof_bringup) should be used. See the package for an in-depth explanation of the launch files that can be used.


## Starting the Farmer and Bull in the arena
The arena has two dynamic obstacles, the farmer and the bull. To start them, follow these instructions:

### Farmer
The farmer is manually controlled by a human. For this, you need another laptop. Connect this laptop to the [Lely Camera network](#connecting-to-the-camera-network).

Once connected, turn on the Farmer robot in the Arena. Wait for it to boot. Once booted, connect to it via SSH:
```bash
$ ssh mirte@192.168.20.199
```
Password is "dev_dev". You are now connected to the `$farmer` shell. Then start the manual control node

```bash
$farmer roslaunch bullproof_control manual_control.launch
```

Now you can control the farmer from another laptop.

### Bull
To start the bull, you can use the same PC as in all other sections (except for [Farmer](#farmer)). Again, make sure you are connected to the [Lely Camera network](#connecting-to-the-camera-network).

Then, connect to the Bull robot via SSH:

```bash
$ ssh mirte@192.168.20.200
```
Password is "dev_dev". You are now connected to the `$bull` shell.

Then, start the bull's pathfinding:

```bash
$bull roslaunch mdp bull.launch
```

## Shutting down the robot
First, shutdown the Bull and Farmer by connecting to their respective shells `$farmer` and `$bull` and running:

```bash
sudo shutdown now
```
To shut down the Mirte robot, run the following command in `$mirte`:

```bash
$mirte sudo shutdown now
```

Once all processes are shutdown, you may  turn off the Mirte robot using the on-board power switch.

**Do not turn it off in any other way. This may cause the entire Mirte robot to be corrupted**



# Packages
This repository contains several packages, which can be started together via the package [bullproof_bringup](#bullproof_bringup) or individually via their own launch files. The following sections describe the packages contained within this repository.

To run any of the launch files, ensure that you are in the workspace folder (bullproof_ws) and have sourced `setup.bash`. To do that, go to the main repository folder (bullproof-tech) and run the following commands:

```bash
cd bullproof_ws
source devel/setup.bash
```

This must be done before running any of the following commands described in the other packages.

## bullproof_bringup
`bullproof_bringup` is responsible for the start-up of the robot or simulation. This package contains the following launch files that the user can use:

### bullproof.launch
This is the main launch file to start the robot. Running this package will start all other required packages for the mirte to be used in normal operation. To start the launch file, run the following:

### bullproof_sim.launch
This launch file starts the simulation of a differential drive robot similar to Mirte in RViz and Gazebo. This smulates the map of the phyisical arena in Robohouse, the farmer and the bull. This can be used to test the navigation stack. The simulation can be started with:
```bash
roslaunch bullproof_bringup bullproof_sim.launch
```
Once started, the robot (white), farmer (red) and bull (blue) will be loaded into Gazebo. 

You have manual control over the farmer. It can be controlled with the arrow keys.

The simulation also launches the state machine from [bullproof_hri](#bullproof_hri). Currently, this state machine can only be triggered to change states manually. The states are as follows:


<p align="center">
<img src="https://drive.google.com/uc?id=1FjvX1z5Dz65cs1aukSxj8IT8ldGVMP9e" width="400"> 
</p>

> Note: The manual changing between states is temporary.
- To toggle between the Clean_Stable and Follow_Farmer state, press E.

- To toggle between the Follow_Farmer and Protect_Farmer state, press R.

- To quit the state machine (not the simulation), press Q.

Pressing R while in the Clean_Stable state will not do anything, you must be in an adjacent state to move towards it.

When in the Clean_Stable state, the Mirte robot will automatically navigate around the stable. When in the Follow_Farmer state, the Mirte robot will follow the farmer (which you control).

> Note: The Protect_Farmer state is not yet implemented.

As an example, in the Follow_Farmer state, the Mirte will start to follow the farmer automatically:
<p align="center">
<img src="https://drive.google.com/uc?id=1qubNvOLLL7IKf-U4dReCadMFlcMEae_g" width="400"> 
</p>

## bullproof_control
`bullproof_control` is responsible for the motion control of the Mirte robot. Since Mirte has an on-board differential drive controller, which can be accessed through the topic `/mobile_base_controller/cmd_vel`, this package only contains a method to manually control the Mirte.
### manual_control.launch
In order to manually control the Mirte robot using arrow keys on your keyboard, first ensure you have the python package `pynput` installed. If not, install it via:

```bash
pip install pynput
```
Then, you can launch `manual_control.launch` as follows:

```bash
roslaunch bullproof_control manual_control.launch
```
While this is running, you will be able to control the Mirte robot.
## bullproof_hri
`bullproof_hri` contains the state machine for the Mirte robot. The state machine determines if the robot is following the farmer, blocking the cow, or helping the farmer exit as well as all other functionality. This state machine is already described in [bullproof_bringup](#bullproof_bringup).

In order to run the state machine, run the following command:
```bash
roslaunch bullproof_hri state_machine.launch
```

Once running, the state machine can be visualised by running:
``` bash
rosrun smach_viewer smach_viewer.py
```

> Note: the state machine is still under development, and is not fully integrated into the simulation yet.
## bullproof_nav
`bullproof_nav` contains the navigation stack that contains Planning for the Mirte robot. This package imports the existing [move_base](http://wiki.ros.org/move_base) package and configurates it using the configuration files from `bullproof_nav/config`. For more information about the node structure of this package, see [Software Architecture](#software-architecture).

### move_base.launch
The [move_base](http://wiki.ros.org/move_base) package allows for moving the robot. This package handles both navigation and motion control. It can be started seperately. To do so, run the following command:
```bash
roslaunch bullproof_nav move_base.launch
```
This is not necessary in general use, as [bullproof_bringup](#bullproof_bringup) already starts this package too, but can be useful for debugging.

## bullproof_perception
`bullproof_perception` contains the perception stack used by this project.

## mirte-ros-packages
`mirte-ros-packages` is a metapackage containing the on-board ROS packages used by the Mirte robot, used for simulating robot in [bullproof_sim.launch](#bullproof_simlaunch).

# Software Architecture
The general software architecture of this project is represented by the following graph:

<img src="https://drive.google.com/uc?id=1UTpOm2Vy6-1HAnEzOcGxgt-OdvMn5Rqz"
alt="node_arch" title="Node Structure"> 

An in-depth explanation of each node is given in the following sections:

## camera_top_view
`camera_top_view` uses the apriltag library and topview camera to get the positions of the bull, the farmer, and the robot. The node has the following in- and outputs:

**Inputs:** 
- No topics as input, this directly takes the camera image

**Outputs:**
- */odom* (nav_msgs/Odometry) - Robot Odometry Pose
- */webcam/image_compressed* (msg_type TBC) - Compressed Overhead Image


## map_server
`map_server`contains the map of the stable containing the obstacles (e.g. stable walls). The node has the following in- and outputs:


**Inputs:** 
- */webcam/image_compressed* (msg_type TBC) - Compressed Overhead Image

**Outputs:**
- */map* (nav_msgs/OccupancyGrid) - Occupancy Grip of the Robot Workspace

## map_update
`map_update`updates the positions of the bull, the farmer and the robot. The node has the following in- and outputs:


**Inputs:** 
- */odom* (nav_msgs/Odometry) - Robot Odometry Pose 
- */map* (nav_msgs/OccupancyGrid) - Occupancy Grip of the Robot Workspace

**Outputs:**
- */map_dynamic* (nav_msgs/GetMap) - Occupancy Map

## farmer_planner
`farmer_planner`publishes cyclical poses for the farmer who patrols the stable. The node has the following in- and outputs:O

**Inputs:** 
- */odom* (nav_msgs/Odometry) - Robot Odometry Pose 

**Outputs:**
- */move_base_simple/goal* (geometry_msgs/PoseStamped) - 2D Nav Goal


## robot_planner
`robot_planner` calculates the optimal location for the robot to be in the space occupied by the farmer and the bull. It also connects to move_base and publishes the optimal location as a 2D NAV goal.

**Inputs:** 
- */odom* (nav_msgs/Odometry) - Robot Odometry Pose 

**Outputs:**
- */move_base_simple/goal* (geometry_msgs/PoseStamped) - 2D Nav Goal

## move_base
The node `move_base` is created by the [move_base](#move_base) package. The node has the following inputs and outputs:

**Inputs:** 
- */map_dynamic* (nav_msgs/GetMap) - Occupancy Map
- */odom* (nav_msgs/Odometry) - Robot Odometry Pose 
- */move_base_simple/goal* (geometry_msgs/PoseStamped) - 2D Nav Goal

**Outputs:**
- */cmd_vel* (geometry_msgs/Twist) - Commanded velocity to robot

A diagram of the navigation stack setup is shown below:

<img src="https://drive.google.com/uc?id=1A0BswrecwWxQcZoPOjms6ZzEVeOoMyzp" alt="move_base" title="Navigation Stack"> 

> Source: http://wiki.ros.org/move_base

For the local planner, the packages is configured to use the `teb_local_planner` This has additional benefits over the basic ROSTrajectoryPlanner, as it allows the robot to move back- as well as forward. For more info, please check out the [teb_local_planner Wiki here](http://wiki.ros.org/teb_local_planner).

The result of this `move_base` implementation can be seen in [this video](https://drive.google.com/uc?id=1aDXfBCpLTQjYPrEhuM-sRDxeSWZhZGoR), where [bullproof_sim.launch](#bullproof_simlaunch) is used to show the Navigation stack in action.

> **NOTE:** While rosdep should take care of dependencies, you might need to install the TEB local planner manually. You can do this using `sudo apt-get install ros-noetic-teb-local-planner`



## contributors
* Jelmer van der Saag
* Nikhil Sethi
* David Janssen
* Tanya Spee
* Dries Borstlap





