#! /usr/bin/env python3


###
# Code adapted from the Kinova Kortex SDK Python examples
###

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading


from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient


from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

try:
    from arm_control.FileRecorder import FileRecorder
except:
    from FileRecorder import FileRecorder


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 30

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


"""
FUNCTIONS FOR MOVING THE ARM
"""

def move_to_home_position(base):
    """
    Moves the Robot Arm to a safe position
    """


    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def move_to_preset_position(base, position_name: str):
    """
    Moves the Robot Arm to a preset position defined in web app (under actions)
    """


    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print(f'Moving the arm to {position_name} position')
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == position_name:
            action_handle = action.handle

    if action_handle == None:
        action_type.action_type = Base_pb2.REACH_POSE
        action_list = base.ReadAllActions(action_type) 
        for action in action_list.action_list:
            if action.name == position_name:
                action_handle = action.handle

    if action_handle == None:
        print("Position not found. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print(f"{position_name} Position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def move_trajectory(base, base_cyclic, waypointsDefinition: Base_pb2.CartesianWaypoint): # type: ignore
    """
    Moves the robot arm to the specified cartesian waypoint
    """

    # Set up
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    waypoints = Base_pb2.WaypointList()
    
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = True
    
    waypoint = waypoints.waypoints.add()
    waypoint.name = "waypoint"   
    waypoint.cartesian_waypoint.CopyFrom(waypointsDefinition)


    # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints)

    if(len(result.trajectory_error_report.trajectory_error_elements) == 0):

        print("Beginning Trajectory ...")
        e_opt = threading.Event()
        notification_handle_opt = base.OnNotificationActionTopic(   check_for_end_or_abort(e_opt),
                                                            Base_pb2.NotificationOptions())

        waypoints.use_optimal_blending = True
        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        finished_opt = e_opt.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle_opt)

        if(finished_opt):
            print("Cartesian trajectory with optimization completed ")
        else:
            print("Timeout on action notification wait for optimized trajectory")

        return finished_opt
        
    else:
        print("Error found in trajectory") 
        result.trajectory_error_report.PrintDebugString();

def move_trajectory_Record(base, base_cyclic, waypointsDefinition: Base_pb2.CartesianWaypoint, file: FileRecorder = None): # type: ignore
    """
    Moves the robot arm to the specified cartesian waypoint
    """
    
    if file is None:
        file = FileRecorder()

    # Set up
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    waypoints = Base_pb2.WaypointList()
    
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = True
    
    waypoint = waypoints.waypoints.add()
    waypoint.name = "waypoint"   
    waypoint.cartesian_waypoint.CopyFrom(waypointsDefinition)


    # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints)

    if(len(result.trajectory_error_report.trajectory_error_elements) == 0):

        print("Beginning Trajectory ...")
        e_opt = threading.Event()
        notification_handle_opt = base.OnNotificationActionTopic(   check_for_end_or_abort(e_opt),
                                                            Base_pb2.NotificationOptions())

        waypoints.use_optimal_blending = True
        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        #finished_opt = e_opt.wait(TIMEOUT_DURATION)
        while(not e_opt.is_set()):
            feedback = base_cyclic.RefreshFeedback()
            file.RecordPosition([feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z])
        base.Unsubscribe(notification_handle_opt)

        finished_opt = e_opt.is_set()

        if(finished_opt):
            print("Cartesian trajectory with optimization completed ")
        else:
            print("Timeout on action notification wait for optimized trajectory")

        return finished_opt
        
    else:
        print("Error found in trajectory") 
        result.trajectory_error_report.PrintDebugString();



"""
FUNCTIONS FOR TESTING
"""

def populateCartesianCoordinate(waypointInformation):
    
    waypoint = Base_pb2.CartesianWaypoint()  
    waypoint.pose.x = waypointInformation[0]
    waypoint.pose.y = waypointInformation[1]
    waypoint.pose.z = waypointInformation[2]
    waypoint.blending_radius = waypointInformation[3]
    waypoint.pose.theta_x = waypointInformation[4]
    waypoint.pose.theta_y = waypointInformation[5]
    waypoint.pose.theta_z = waypointInformation[6] 
    waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    
    return waypoint

def get_coordinates_from_user():
    # Get the x, y, z coordinates from the user
    coordinates = input("Enter the x, y, z coordinates (separated by spaces): ").split()
    x = float(coordinates[0])
    y = float(coordinates[1])
    z = float(coordinates[2])
    return (x, y, z)

def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        
        # Example core
        success = True

        success &= move_to_home_position(base)

        while True:
            # Get the coordinates from the user
            coordinates = get_coordinates_from_user()

            # Assuming base_cyclic is your BaseCyclic object and you've already connected to the robot
            feedback = base_cyclic.RefreshFeedback()

            # Update the waypointsDefinition with the new coordinates
            waypointsDefinition = (coordinates[2], coordinates[0], coordinates[1], 0.0, 90.0, 0.0, 90.0)
            waypointsDefinition = populateCartesianCoordinate(waypointsDefinition)

            success &= move_trajectory(base, base_cyclic, waypointsDefinition)

            feedback = base_cyclic.RefreshFeedback()

            arm_x = feedback.base.tool_pose_x
            arm_y = feedback.base.tool_pose_y
            arm_z = feedback.base.tool_pose_z


            print(f"Arm moved to: {arm_x}, {arm_y}, {arm_z}")
       
        return 0 if success else 1
    
def test():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        
        # Example core
        success = True

        success &= move_to_home_position(base)

        success &= move_to_preset_position(base, "Home Left")

        waypointsDefinition = (0.5, 0.3, 0.7, 0.0, 90.0, 0.0, 90.0)
        waypointsDefinition = populateCartesianCoordinate(waypointsDefinition)

        success &= move_trajectory_Record(base, base_cyclic, waypointsDefinition)

        return 0 if success else 1

if __name__ == "__main__":
    exit(test())