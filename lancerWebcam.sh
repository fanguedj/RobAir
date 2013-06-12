 #!/bin/bash
 gnome-terminal -x bash -c "roscore"
sleep 3
export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,width=320,height=240,framerate=(fraction)30/1 ! ffmpegcolorspace"
 gnome-terminal -x bash -c "rosrun gscam gscam"
 gnome-terminal -x bash -c "rosrun mjpeg_server mjpeg_server"
