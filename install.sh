#!/bin/sh

# simple script to get the raspberry pi ready for testing.  make sure
# to 'sudo' this script.

# uninstall some other stuff
rmmod -f gspca_kinect gspca_main videodev

# uninstall the rpi-kinect in case it is loaded
rmmod -f rpi-kinect

# install the module
insmod ./rpi-kinect.ko debug_level=31
