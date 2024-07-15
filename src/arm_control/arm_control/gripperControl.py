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
    def __init__(self, router):

        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def GripApple(self):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_request = Base_pb2.GripperRequest()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        gripper_request.mode = Base_pb2.GRIPPER_POSITION

        gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)

        position = gripper_measure.finger[0].value
        finger.finger_identifier = 1
        while position < 1.0:
            
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value <= position - 0.008:
                    
                    break
            else: # Else, no finger present in answer, end loop
                break

            position += 0.02

            finger.value = position
            print("Going to position {:0.5f}...".format(finger.value))
            self.base.SendGripperCommand(gripper_command)
            
            time.sleep(0.1)

    def OpenGripper(self):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        # Set grippper to open position
        print ("Opening gripper using position command...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.value = 0.0
        self.base.SendGripperCommand(gripper_command)
        

def main():
    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        example = GripperCommand(router)
        example.GripApple()

if __name__ == "__main__":
    main()