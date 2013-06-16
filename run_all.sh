 #!/bin/bash
export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,width=640,height=480,framerate=(fraction)15/1 ! videoflip method=counterclockwise ! ffmpegcolorspace"
 gnome-terminal -x bash -c "roslaunch robair_demo run_all.launch"
 gnome-terminal -x bash -c "cd /home/fablab/ros_workspace/robair_demo/scripts/; ./robair_print_pic.py static/images/photo.jpg"
 
