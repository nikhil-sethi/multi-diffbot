# BullProof Techologies

MDP Project of 2022/2023 Q4, Group 8, Lely

# Installation, Setup and Robot Shutdown
## Cloning repository
To clone this repository, please open a terminal in the folder where you would like to create the repository and run the following command:

``` bash
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-8/bullproof-tech.
```
The repository should now be cloned on your machine.
## Build
To build the repository, open the `bullproof-tech` folder in a terminal and run the following commands:

``` bash
git submodule update --init --recursive
cd bullproof_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
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
$ export ROS_MASTER_URI=https://192.168.42.1:11311
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


# bullproof-control
This repository contains a package named bullproof-control. This package takes care of the motion control of the Mirte robot. 

## manual_control.launch
In order to manually control the Mirte robot using arrow keys on your keyboard, you can launch `manual_control.launch` as follows:

```bash
cd bullproof_ws
source devel/setup.bash
roslaunch bullproof-control manual_control.launch
```

## Simulation

Useful to simulate the map of robohouse. 
Planning with movebase using 2D NAV goals is supported.
```bash
roslaunch bullproof-bringup bullproof_sim.launch
```

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