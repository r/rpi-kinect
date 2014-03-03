a simple kernel module to talk to the kinect from a raspberry pi  
raffi.krikorian@gmail.com


this module is incredibly simple. all it supports is moving the kinect
motor (writing to /dev/kinect%d), reading the motor status, and
reading the accelerometers (reading from /dev/kinect%d).

