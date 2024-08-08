# Nodes

## move_tank

This node interfaces with the tank microcontroller via usb serial. A specific usb port is used. When a message is received on the move_tank_command topic, then a connection is made through usb serial and the motor speeds are set

The port the connect to the tank is /dev/ttyRoboteq (udev setting on Jetson using vendor ID and product ID)

# Other Functions

## RoboteqDevice.cpp

This file contains a class to connect to and send commands to the tank microcontroller



