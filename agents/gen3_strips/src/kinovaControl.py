#! /usr/bin/env python3

###
#
# Kinova Gen3 Control class
#
###

import sys
import os
import time
import threading

import utilities
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

class KinovaGen3():
    def __init__(self):
        
        ip = "192.168.1.10"
        username = "admin"
        psw = "admin"

        self.connection = utilities.DeviceConnection.createTcpConnectionExplicit(ip, username, psw)
        self.router = self.connection.connect()

        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)
    
    def __del__(self):
        """Destructor"""
        self.connection.disconect()

    # Create closure to set an event after an END or an ABORT
    def check_for_end_or_abort(self, e):
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

    def get_state (self):
        feedback = self.base_cyclic.RefreshFeedback()
        return feedback.base

    def get_gripper_state (self):
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        return self.base.GetMeasuredGripperMovement(gripper_request).finger[0].value

    def get_pose (self):
        state = self.get_state()

        return [state.tool_pose_x,       state.tool_pose_y,       state.tool_pose_z, 
                state.tool_pose_theta_x, state.tool_pose_theta_y, state.tool_pose_theta_z]

        # return {"x": state.tool_pose_x,
        #         "y": state.tool_pose_y,
        #         "z": state.tool_pose_z,
                
        #         "theta_x": state.tool_pose_theta_x,
        #         "theta_y": state.tool_pose_theta_y,
        #         "theta_z": state.tool_pose_theta_z
        #         }

    def cartesian_move_to(self, x, y, z, theta_x, theta_y, theta_z):
    
        print("Starting Cartesian Especific Movement ...")
        action = Base_pb2.Action()
        action.name = "Cartesian Especific movement"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x        =  x        # (meters)
        cartesian_pose.y        =  y        # (meters)
        cartesian_pose.z        =  z        # (meters)
        cartesian_pose.theta_x  =  theta_x  # (degrees)
        cartesian_pose.theta_y  =  theta_y  # (degrees)
        cartesian_pose.theta_z  =  theta_z  # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def cartesian_move_relative(self, x, y, z, theta_x, theta_y, theta_z):
    
        print("Starting Cartesian action movement ...")
        action = Base_pb2.Action()
        action.name = "Example Cartesian action movement"
        action.application_data = ""

        feedback = self.base_cyclic.RefreshFeedback()

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x        = feedback.base.tool_pose_x + x # (meters)
        cartesian_pose.y        = feedback.base.tool_pose_y + y # (meters)
        cartesian_pose.z        = feedback.base.tool_pose_z + z # (meters)
        cartesian_pose.theta_x  = feedback.base.tool_pose_theta_x + theta_x # (degrees)
        cartesian_pose.theta_y  = feedback.base.tool_pose_theta_y + theta_y # (degrees)
        cartesian_pose.theta_z  = feedback.base.tool_pose_theta_z + theta_z # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def move_gripper_speed_dest (self, dest_pos):

        gr_pos = self.get_gripper_state()

        direction = 1 if (gr_pos - dest_pos) > 0 else -1

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        # Set speed to close gripper
        print ("Closing gripper using speed command to dest pos")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1 * direction
        self.base.SendGripperCommand(gripper_command)



        vel_gripper_request = Base_pb2.GripperRequest()
        # Wait for reported speed to be 0
        vel_gripper_request.mode = Base_pb2.GRIPPER_SPEED

        pos_gripper_request = Base_pb2.GripperRequest()
        # Wait for reported pos to be dest_pos
        pos_gripper_request.mode = Base_pb2.GRIPPER_POSITION
        
        # Speed is initially 0, to avoid premature stopping: 
        time.sleep(.1)

        vel_gripper_measure = self.base.GetMeasuredGripperMovement(vel_gripper_request)
        pos_gripper_measure = self.base.GetMeasuredGripperMovement(pos_gripper_request)
        while abs(dest_pos - pos_gripper_measure.finger[0].value) > 0.01 and \
                             vel_gripper_measure.finger[0].value != 0.00:
            vel_gripper_measure = self.base.GetMeasuredGripperMovement(vel_gripper_request)
            pos_gripper_measure = self.base.GetMeasuredGripperMovement(pos_gripper_request)

        finger.value = 0.0
        self.base.SendGripperCommand(gripper_command)


    def close_gripper_speed (self):

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        # Set speed to close gripper
        print ("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = -0.1
        self.base.SendGripperCommand(gripper_command)

        gripper_request = Base_pb2.GripperRequest()
        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_SPEED
        
        # Speed is initially 0, to avoid premature stopping: 
        time.sleep(.1)
        while True:
            gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value == 0.0:
                    break
            else: # Else, no finger present in answer, end loop
                break

    def open_gripper_speed (self):

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Set speed to open gripper
        print ("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.01:
                    break
            else: # Else, no finger present in answer, end loop
                break