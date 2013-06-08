#!/bin/bash
gnome-terminal -x bash -c "roscore; sleep 10000"
sleep 2
gnome-terminal -x bash -c "rosrun robair_demo arduino_sensors.py; sleep 10000"
gnome-terminal -x bash -c "rosrun hokuyo_node hokuyo_node; sleep 10000"
gnome-terminal -x bash -c "rosrun robair_demo odometry; sleep 10000"
sleep 2
roslaunch robair_demo slamGmappingLidar.launch
