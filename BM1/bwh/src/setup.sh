#! /bin/bash

echo "Setting Keyboard Control"

mkdir test_ws && cd test_ws
mkdir src && catkin init 
cd src && catkin_create_pkg keyboard_control roscpp rospy trajectory_msgs
cd keyboard_control/src

mv ../..keyboard_control.cpp keyboard_control/src/ 
