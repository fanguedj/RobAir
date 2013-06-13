 #!/bin/bash
 gnome-terminal -x bash -c "roscore; sleep 10000"
sleep 2
 gnome-terminal -x bash -c "rosrun robair_demo kb_control.py; sleep 10000"
 gnome-terminal -x bash -c "rosrun robair_demo motion_control_node.py; sleep 10000"
 gnome-terminal -x bash -c "rosrun robair_demo arduino_sensors.py; sleep 10000"
 gnome-terminal -x bash -c "cd /home/fablab/ros_workspace/robair_demo/scripts; ./serverWeb.py; sleep 10000"
sleep 3
export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,width=320,height=240,framerate=(fraction)30/1 ! ffmpegcolorspace"
 gnome-terminal -x bash -c "rosrun gscam gscam"
 gnome-terminal -x bash -c "rosrun mjpeg_server mjpeg_server"
 gnome-terminal -x bash -c "cd /home/fablab/ros_workspace/robair_demo/scripts/; ./robair_print_pic.py static/images/photo.jpg"
