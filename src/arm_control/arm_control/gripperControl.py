#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the
# terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

class GripperCommand:
    """
    The GripperCommand class provides control over the robot's gripper.

    It allows you to grip an apple and open the gripper using position commands.

    Args:
        router: The router object used for communication with the robot.

    Functions:
        OpenGripper() - Opens the gripper to the fully open position.
        GripApple() - Grips an apple by closing the gripper with position increments.
                        If the commanded position is not reached (something is blocking it), the gripper will stop closing.

    Example usage:
    ```
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        example = GripperCommand(router)
        example.OpenGripper()
        example.GripApple()
    ```

    """

    def __init__(self, router):
        self.router = router
        self.base = BaseClient(self.router)

    def GripApple(self):
        """
        Grips an apple by closing the gripper with position increments.
        """
        gripper_command = Base_pb2.GripperCommand() # Create a gripper command
        finger = gripper_command.gripper.finger.add()
        gripper_request = Base_pb2.GripperRequest() # Create a gripper request (for information)

        gripper_command.mode = Base_pb2.GRIPPER_POSITION # command mode is position
        gripper_request.mode = Base_pb2.GRIPPER_POSITION # feedback mode is position

        gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request) # Get the current gripper position

        position = gripper_measure.finger[0].value
        finger.finger_identifier = 1
        while position < 0.98:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request) # Refresh gripper position
            if len(gripper_measure.finger):
                if gripper_measure.finger[0].value <= position - 0.01: # If the gripper does not reach position (within tolerance) it is blocked: stop closing
                    break
            else:
                break

            position += 0.02    # Increment position
            finger.value = position # Set the new position
            self.base.SendGripperCommand(gripper_command) # Send the command
            time.sleep(0.1) # Give the gripper a chance to reach the commanded position
        
        return


    def OpenGripper(self):
        """
        Opens the gripper to the fully open position.
        """
        gripper_command = Base_pb2.GripperCommand() # Create a gripper command
        gripper_request = Base_pb2.GripperRequest() # Create a gripper request (for information)
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION # command mode is position
        gripper_request.mode = Base_pb2.GRIPPER_POSITION # feedback mode is position
        finger.value = 0.0
        self.base.SendGripperCommand(gripper_command)
        # up to ten checks to see if the gripper is fully open before giving up
        for _ in range(10):
            if self.base.GetMeasuredGripperMovement(gripper_request).finger[0].value < 0.01:
                break # Gripper is fully open
            else:
                time.sleep(0.1) # Wait a bit before checking again
        return
            
            

def main():
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        example = GripperCommand(router)
        example.OpenGripper()
        example.GripApple()

if __name__ == "__main__":
    main()