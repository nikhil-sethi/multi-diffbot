# BullProof Techologies
Team Logo:

<img src="https://drive.google.com/uc?id=1R4BYatSzIrdLh8kk5JZJNoghUWQ8TwpW" alt="Team Logo" width="50%" height="50%" title="Team Logo"> 

This repostory contains the Multi-Disciplinary Project of 2022/2023 of Group 8, developed for Lely. This ReadMe file contains instructions to install and set up the robot, information about the packages contained in the repository and a brief overview of the software architecture used in the robot.


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
    - [ldlidar_stl_ros](#ldlidar_stl_ros)
    - [mirte-ros-packages](#mirte-ros-packages)
- [Software Architecture](#software-architecture)
    - [move_base](#move_base)
- [ToDo's](#todos)
# Installation, Setup and Robot Shutdown
## Cloning repository
To clone this repository, please open a terminal in the folder where you would like to create the repository and run the following command:

``` bash
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-8/bullproof-tech.
```
The repository should now be cloned on your machine.
## Building the workspace
To build the repository, open the `bullproof-tech` folder in a terminal and run the following commands:

``` bash
git submodule update --init --recursive
cd bullproof_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

> **NOTE:** Make sure you are using rosdep for ROS Noetic! Any other version may result in unwanted consequences.

This will update any submodules, install dependencies and then build the workspace for ROS within the workspace folder `bullproof_ws`.

## Connecting to the robot
Firstly, one must connect to the WiFi network that the Mirte robot creates. Look for and connect to the following WiFi network:
* Network: mirte_bullproof
* Password: mirte_mirte

Once connected, one can connect to the `$mirte` shell via ssh by running the following commands:

```bash
$mirte ssh mirte@192.168.42.1
```
It will then ask for a password, this password is "bullproof". Once connected, you now have access to the `$mirte` shell. This is required to shut the robot down later.

## Connecting to the camera
(coming soon)

## Starting the robot
In order to start up the robot, the package `bullproof-bringup` should be used. See the section [bullproof-bringup](#bullproof_bringup) for more information on how to start up the robot.

In order to connect to the ROS master running on Mirte, open a new terminal and run the following in the user shell `$` (not `$mirte`):

```bash
$ export ROS_MASTER_URI=http://192.168.42.1:11311
$ export ROS_IP=<your_wlan_ip>
```
If you do not know what your WLAN IP address is, run `$ ifconfig` or `$ hostname -I` to check.

Once connected, you can now execute any scripts or commands to the ROS master on the Mirte robot. This must be re-done for every new shell instance. You do not need to run `$roscore` locally.

## Shutting down the robot
To shut down mirte, run the following command in the `$mirte`:

```bash
$mirte sudo shutdown now
```

Once all processes are shutdown, you may  turn off the Mirte robot using the on-board power switch.

**Do not turn it off in any other way. This may cause the entire Mirte robot to be corrupted**



# Packages
This repository contains several packages, which can be started together via the package [bullproof_bringup](#bullproof_bringup) or individually via their own launch files. The following sections describe the packages contained within this repository.

To run any of the launch files, ensure that you are in the workspace and have sourced `setup.bash` by running the following commands:

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
Once started, the robot and farmer can be controlled using 2D navigation goals.

## bullproof_control
`bullproof_control` is responsible for the motion control of the Mirte robot. Since Mirte has an on-board differential drive controller, which can be accessed through the topic `/mobile_base_controller/cmd_vel`, this package only contains a method to manually control the Mirte.
### manual_control.launch
In order to manually control the Mirte robot using arrow keys on your keyboard, you can launch `manual_control.launch` as follows:

```bash
roslaunch bullproof_control manual_control.launch
```
While this is running, you will be able to control the Mirte robot.
## bullproof_hri
`bullproof_hri` contains the behavioural tree for the Mirte robot. More information to be added.
## bullproof_nav
`bullproof_nav` contains the navigation stack that contains Planning for the Mirte robot. This package imports the existing [move_base](http://wiki.ros.org/move_base) package and configurates it using the configuration files from `bullproof_nav/config`. For more information about the node structure of this package, see [Software Architecture](#software-architecture).

### move_base.launch
The [move_base](http://wiki.ros.org/move_base) package can be started seperately. To do so, run the following command:
```bash
roslaunch bullproof_nav move_base.launch
```
This is not necessary in general use, as [bullproof_bringup](#bullproof_bringup) already starts this package too, but can be useful for debugging.

## bullproof_perception
`bullproof_perception` contains the perception stack used by this project. More information to be added.

## ldlidar_stl_ros
Submodule containing the packages for the LIDAR on-board the Mirte robot. Cannot be started seperately.
## mirte-ros-packages
Metapackage containing the on-board ROS packages used by the Mirte robot, used for simulating robot in [bullproof_sim.launch](#bullproof_simlaunch).

# Software Architecture
To be added: overview of node structure and software architecture.

## move_base
The node `move_base` is created by the [move_base](#move_base) package. The nodes has the following inputs and outputs

**Inputs:** 
- */map* (nav_msgs/GetMap) - Occupancy Map
- */odomo* (nav_msgs/Odometry) - Robot Odometry Pose
- */move_base_simple/goal* (geometry_msgs/PoseStamped) - 2D Nav Goal

**Outputs:**
- */cmd_vel* (geometry_msgs/Twist) - Commanded velocity to robot

A diagram of the navigation stack setup is shown below:

<img src="https://drive.google.com/uc?id=1A0BswrecwWxQcZoPOjms6ZzEVeOoMyzp
" alt="move_base" title="Navigation Stack"> 



# ToDo's

-  Planning
    - Optimal location calculation algorithm
    - Tune movebase params to match real mirte dynamics 
- Perception
    - Detection algo
    - Publish map + poses on required topics
- HRI
    - Behaviour trees
    - Handle mirte overtake request
- Control
- Simulation
    - Add farmer and bull robots + topics
- Finalise ReadMe documentation with finished packages and camera instructions