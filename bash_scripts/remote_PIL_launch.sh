#!/bin/bash
(ssh pi@raspberrypi.local 'source /home/pi/catkin_ws/devel/setup.bash; roscore')&

rsync -av -e ssh --exclude='submodule' ~/catkin_ws/src/generic_gnc pi@raspberrypi.local:~/catkin_ws/src/
ssh pi@raspberrypi.local 'cp ~/catkin_ws/src/generic_gnc/bash_scripts/remote_env_loader.sh ~/catkin_ws/devel'
echo "OK"
if [[ $1 == "c" ]]; then
    ssh pi@raspberrypi.local 'source /home/pi/catkin_ws/devel/setup.bash;cd ~/catkin_ws;catkin_make'
fi


export ROS_IP=192.168.43.31
export ROS_MASTER_URI=http://192.168.43.127:11311

roslaunch generic_gnc generic_PIL.launch
