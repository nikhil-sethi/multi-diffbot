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

## Connecting to the Robot
Firstly, one must connect to the WiFi network that the Mirte robot creates. Look for and connect to the following WiFi network:
* Network: mirte_bullproof
* Password: mirte_mirte

Once connected, one can connect to the `$mirte` shell via ssh by running the following commands:

```bash
ssh mirte@192.168.42.1
```
It will then ask for a password, this password is "bullproof". Once connected, you now have access to the `$mirte` shell.