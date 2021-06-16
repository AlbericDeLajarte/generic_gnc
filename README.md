# generic_gnc
This package provides a navigation and a control algorithms to control the altitude of the Bellalui 2 rocket.
It is supposed to be deployed on a raspberry pi connected to the rocket's avionics, but can also be installed on a personal computer (Ubuntu 18) for simulation in the loop tests.

# Installation
*This installation supposes that you already have ROS melodic installed on Ubuntu or Raspian, and a working catkin workspace*

1. Install the simulator:
The package [real_time_simulator](https://github.com/EPFLRocketTeam/real_time_simulator) contains all the utilities and definitions needed to use these GNC algorithms. Check the related github page for its installation.

If you want to deploy it on a raspberrypi and have probalems at compilation, you can comment out the executables in CMakeLists.txt, as they are not needed to run the GNC algorithms.

2. Install the GNC package:
```bash
cd ~/catkin_ws/src
git clone https://github.com/EPFLRocketTeam/generic_gnc.git
cd ..
catkin_make
```

3. Test installation:
*If installed on a personal computer*
```bash
roslaunch generic_gnc generic_SIL.launch
```

Visualize the result of the simulation:
```bash
roscd real_time_simulator/postProcess/
python postProcess.py
```


*If deployed on a raspberry pi*
```bash
roslaunch generic_gnc generic_PIL.launch
```

