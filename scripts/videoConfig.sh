#!/bin/bash

export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,width=320,height=240,framerate=(fraction)10/1 ! ffmpegcolorspace"
