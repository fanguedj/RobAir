#!/bin/bash

toFind=$1

if [ -z "$toFind" ]
then
    toFind="arduino"
fi

default="/ttyACM0"

top() {
    echo "################ Serial Port ################" > /dev/stderr
}

bottom() {
    echo "#############################################" > /dev/stderr
}

warning() {

    top
    echo "Warning : serial port for "$toFind" not found. Default /dev"$default" sent."> /dev/stderr;
    echo "Check if device "$toFind" is connected." > /dev/stderr
    port=$default
    bottom
}

noSerialAndQuit() {
    top
    echo "Error : no serial port found. Check connections." > /dev/stderr
    bottom
    exit 1
}

(ls -l /dev/serial/ > /dev/null ) || noSerialAndQuit

port=`(ls -l /dev/serial/by-id/ | grep $toFind | grep -oE /tty.*) 2> /dev/null`

if [ -z "$port" ] #port NULL
then 
  warning
fi

port="/dev"$port

echo $port

exit 0
