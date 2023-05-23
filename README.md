# BullProof Techologies

MDP Project of 2022/2023 Q4, Group 8, Lely

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
catkin_make
```
This will update any submodules and then build the workspace for ROS within the workspace folder `bullproof_ws`.

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
$ export ROS_MASTER_URI=https://192.168.42.1/11311
$ export ROS_IP=<your_wlan_ip>
```
If you do not know what your WLAN IP address is, run `$ ifconfig` or `$ hostname -I` to check.

Once connected, you can now execute any scripts or commands to the ROS master on the Mirte robot. This must be re-done for every new shell instance. You do not need to run `$roscore` locally.

## Shutting down the robot
To shut down mirte, run the following command in the `$mirte`:

```bash
$mirte sudo shutdown now
```

Once all processes are shutdown, you may remove the power or turn off the onboard Orange Pi. 

**Do not turn it off in any other way. This may cause the entire Mirte robot to be corrupted**