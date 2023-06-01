# BullProof Techologies

This repostory contains the Multi-Disciplinary Project of 2022/2023 Q4, Group 8, Lely.

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

> **NOTE:** Make sure you are using rosdep for ROS noetic! Any other version may result in unwanted consequences.

This will update any submodules, install dependencies and then build the workspace for ROS within the workspace folder `bullproof_ws`.

## Connecting to the robot
Firstly, one must connect to the WiFi network that the Mirte robot creates. Look for and connect to the following WiFi network:
* Network: mirte_bullproof
* Password: mirte_mirte

Once connected, one can connect to the `$mirte` shell via ssh by running the following commands:

```bash
$mirte ssh mirte@192.168.42.1
```
It will then ask for a password, this password is "bullproof". Once connected, you now have access to the `$mirte` shell.

In order to connect to the ROS master running on Mirte (in order to publish or subscribe to its nodes), the following should be run in the user shell `$` (not `$mirte`):

```bash
$ export ROS_MASTER_URI=http://192.168.42.1:11311
$ export ROS_IP=<your_wlan_ip>
```
If you do not know what your WLAN IP address is, run `$ ifconfig` or `$ hostname -I` to check.

Once connected, you can now execute any scripts or commands to the ROS master on the Mirte robot. This must be re-done for every new shell instance. You do not need to run `$roscore` locally.

## Starting the robot
In order to start up the robot, the package `bullproof-bringup` should be used. See the section [bullproof-bringup](#bullproof_bringup) for more information on how to start up the robot.

## Shutting down the robot
To shut down mirte, run the following command in the `$mirte`:

```bash
$mirte sudo shutdown now
```

Once all processes are shutdown, you may  turn off the Mirte robot using the on-board power switch.

**Do not turn it off in any other way. This may cause the entire Mirte robot to be corrupted**



# Packages
This repository contains several packages, which can be started together via the package `bullproof_bringup` or individually. The following sections describe the packages contained within this repository

## bullproof_bringup
bullproof_bringup is responsible for the start-up of the robot or simulation. This package can be started with the following launch files:

### bullproof_sim.launch
This launch file starts the simulation of a differential drive robot similar to Mirte in RViz and Gazebo. This smulates the map of the phyisical arena in Robohouse, and can be used to test the navigation stack. The simulation can be started with:
```bash
roslaunch bullproof_bringup bullproof_sim.launch
```
Once started, the robot can be controlled using 2D navigation goals.

## bullproof_control
### manual_control.launch
In order to manually control the Mirte robot using arrow keys on your keyboard, you can launch `manual_control.launch` as follows:

```bash
cd bullproof_ws
source devel/setup.bash
roslaunch bullproof_control manual_control.launch
```

## bullproof_hri

## bulproof_nav

## bullproof_perception
## ldlidar_stl_ros
Submodule containing the packages for the LIDAR on-board the Mirte robot. Cannot be started seperately.
## mirte-ros-packages
Metapackage containing the on-board ROS packages used by the Mirte robot, used for simulating robot.
## TODO

- [] Planning
    - [] Optimal location calculation algorithm
    - [] Tune movebase params to match real mirte dynamics 
- [] Perception
    - [] Detection algo
    - [] Publish map + poses on required topics
- [] HRI
    - [] Behaviour trees
    - [] Handle mirte overtake request
- [] Control
- [] Simulation
    - [] Add farmer and bull robots + topics
    - [] 