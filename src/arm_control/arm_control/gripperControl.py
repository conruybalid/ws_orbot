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
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_request = Base_pb2.GripperRequest()

        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        gripper_request.mode = Base_pb2.GRIPPER_POSITION

        gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)

        position = gripper_measure.finger[0].value
        finger.finger_identifier = 1
        while position < 1.0:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                if gripper_measure.finger[0].value <= position - 0.008:
                    break
            else:
                break

            position += 0.02
            finger.value = position
            self.base.SendGripperCommand(gripper_command)
            time.sleep(0.1)

    def OpenGripper(self):
        """
        Opens the gripper to the fully open position.
        """
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.value = 0.0
        self.base.SendGripperCommand(gripper_command)

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