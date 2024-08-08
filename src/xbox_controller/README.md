# Nodes

## controller_node

This node looks for controller inputs (bluetooth or wired). It translates joystick inputs into acceptable motor inputs. When the manual_control flag is true (toggled with A on the controller), it publishes to the move_tank_command topic

When the manual_control flag is false, the buttons can be used to move the arm:
- (Y): Move arm to home position
- (X): Move arm to left down position
- (B): Move arm to Right down position
- (left D-pad): Close gripper
- (right D-pad): Open gripper 

# Other Functions

## XboxControllerTest.py

A script that allows testing of the controller inputs



