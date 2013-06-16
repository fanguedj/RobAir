#!/bin/bash

export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,width=640,height=480,framerate=(fraction)15/1 ! videoflip method=counterclockwise ! ffmpegcolorspace"
rosrun gscam gscam
