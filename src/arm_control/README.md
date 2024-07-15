# Nodes

## arm_move_action

This node creates an action that uses the ArmMove action from custom_interfaces. This action receives a ArmControl goal and attempts to move the arm to the specified waypoint and changes the gripper state. It provides string feedback info and a true or false result


# Other Functions

## mytry.py

This file contains a function to move the arm used in the arm_move_action node. It can be run on it's own to test waypoints.

## gripperControl.py

This file contains a class that is used for opening and closing the gripper. It can be run on it's own for testing

## utilities.py

This file contains DeviceConnection class used to connect to the Kinova Kortex Arm


