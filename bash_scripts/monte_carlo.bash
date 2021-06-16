#!/usr/bin/env bash
source ~/catkin_ws/devel/setup.bash

for ((a=0; a < 1000; a++))
do
    echo $(($a+1))
    (roslaunch generic_gnc monte_carlo_parameters.launch >/dev/null)&
    sleep 2

    (roslaunch generic_gnc monte_carlo_nodes.launch >/dev/null)&
    sleep 9

    (rostopic pub /commands std_msgs/String "data: ''"  >/dev/null)&

    sleep 33

    rosnode kill generic_control >/dev/null
    rosnode kill generic_navigation >/dev/null
    rosnode kill generic_fsm >/dev/null
    rosnode kill av_interface >/dev/null
    rosnode kill aerodynamic >/dev/null
    rosnode kill disturbance >/dev/null
    rosnode kill integrator >/dev/null

    kill $(jobs -p) >/dev/null

    sleep 2

done
