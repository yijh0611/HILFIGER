#!/bin/bash

# ./hak.sh 하지 말고 source ./hak.sh를 해야 한다.
export DISPLAY='165.194.85.129:0.0'
echo 'Display changed'

catkin_make
chmod +x devel/setup.sh
source devel/setup.sh

chmod +x -R src/

cp session_hak.yml ../startup/challenge/session_hak.yml
cp start_hak.sh ../startup/challenge/start_hak.sh

cd ../startup/challenge/
source start_hak.sh

# ./start_hak.sh
# cd ..
# cd startup/challenge/

# echo 'End'
