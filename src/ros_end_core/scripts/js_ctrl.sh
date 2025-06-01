# before source this script, make sure to run the following command:
# cd /media/mitchell/Data/Projects/software_engineering/ros_end 
# source devel/setup.bash
# roslaunch ros_end_core full.launch

#!/bin/bash
# This script is used to control the robot using the joystick.
# It is used to test the robot's movement and functionality, only admin can run this script.
rosrun joy joy_node _dev:=/dev/input/js0 _deadzone:=0.12 __name:=wpb_home_joy __respawn:=true