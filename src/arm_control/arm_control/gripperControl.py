import utilities

import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2


def gripper_control(base, close):
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    gripper_request = Base_pb2.GripperRequest()


    if not close:
        # Set speed to open gripper
        print ("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        base.SendGripperCommand(gripper_command)

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.01:
                    break
            else: # Else, no finger present in answer, end loop
                break

    else:
        max_force = 1.0
        # Set speed to close gripper
        print ("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = -0.1
        base.SendGripperCommand(gripper_command)

        time.sleep(0.02)

        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_SPEED
        while True:
            gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current Speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value >= 0.0:
                    break
            else: # Else, no finger present in answer, end loop
                break


args = utilities.parseConnectionArguments()

# Create connection to the device and get the router
with utilities.DeviceConnection.createTcpConnection(args) as router:

    base = BaseClient(router)

    gripper_control(base, True)