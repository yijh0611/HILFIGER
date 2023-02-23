#!/bin/bash

# ./hak.sh 하지 말고 source ./hak.sh를 해야 한다.
export DISPLAY='165.194.85.129:0.0'
echo 'Display changed'

catkin_make
chmod +x devel/setup.sh
source devel/setup.sh

chmod +x src/carrot_team/src/control.py

cd ..
cd startup/challenge/

echo 'End'