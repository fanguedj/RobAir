#!/bin/bash

ifconfig | grep -E -A 1  wlan0 | grep -oE adr:[0-9.]* | grep -oE [0-9.]*

