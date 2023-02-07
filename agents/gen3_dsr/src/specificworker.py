#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from ntpath import join
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
from scipy.spatial.transform import Rotation as R

# import kinovaControl

import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 1

        self.agent_id = 30
        self.g = DSRGraph(0, "gen3 DSR", self.agent_id)
        self.rt = rt_api(self.g)

        # self.arm = kinovaControl.KinovaGen3()

        self.HAND_CAMERA_SN = '037322251488'

        # Hand camera configuration
        self.pipeline_hand = rs.pipeline()
        config_hand = rs.config()
        config_hand.enable_device(self.HAND_CAMERA_SN)
        config_hand.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config_hand.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline_hand.start(config_hand)

        self.camera_node = self.g.get_node('hand_camera')
        
        if self.camera_node is not None:
            self.camera_node.attrs['cam_rgb'         ] = Attribute (np.zeros((480,640,3), np.uint8), self.agent_id)
            self.camera_node.attrs['cam_rgb_width'   ] = Attribute (640,        self.agent_id)
            self.camera_node.attrs['cam_rgb_height'  ] = Attribute (480,        self.agent_id)
            self.camera_node.attrs['cam_rgb_depth'   ] = Attribute (3,          self.agent_id)
            self.camera_node.attrs['cam_rgb_focalx'  ] = Attribute (380.841857910156,  self.agent_id)
            self.camera_node.attrs['cam_rgb_focaly'  ] = Attribute (379.866882324219    , self.agent_id)
            self.camera_node.attrs['cam_rgb_cameraID'] = Attribute (0,          self.agent_id)

            self.camera_node.attrs['cam_depth'         ] = Attribute (np.zeros((480,640,2), np.uint8), self.agent_id)
            self.camera_node.attrs['cam_depth_width'   ] = Attribute (480,           self.agent_id)
            self.camera_node.attrs['cam_depth_height'  ] = Attribute (640,           self.agent_id)
            self.camera_node.attrs['cam_depth_depth'   ] = Attribute (2,             self.agent_id)
            self.camera_node.attrs['cam_depth_focalx'  ] = Attribute (392.261108398438,     self.agent_id)
            self.camera_node.attrs['cam_depth_focaly'  ] = Attribute (392.261108398438, self.agent_id)
            self.camera_node.attrs['cam_depthFactor'   ] = Attribute (0.01,          self.agent_id)
            self.camera_node.attrs['cam_depth_cameraID'] = Attribute (1,             self.agent_id)
            


            self.g.update_node(self.camera_node)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.gripper = self.g.get_node('gripper')
        self.gripper.attrs['gripper_finger_distance'] = Attribute (float(0.0), self.agent_id)
        self.gripper.attrs['gripper_target_finger_distance'] = Attribute (float(0.0), self.agent_id)
        self.gripper.attrs['robot_occupied'] = Attribute (False, self.agent_id)
        self.g.update_node(self.gripper)

        self.arm_node = self.g.get_node ('arm')
        self.arm_node.attrs['robot_local_angular_velocity'] = Attribute ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], self.agent_id)
        self.g.update_node (self.arm_node)

        self.GRIPPER_ID = self.gripper.id

        self.moving = False
        self.target_position = None
        self.last_target_position = None
        self.occupied = False

        self.gripper_target_position = None
        self.gripper_last_target_position = None

        # th = threading.Thread(target=self.arm_mover_thread)
        # th.start()


        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            # signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            # signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            # signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            # signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            # signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""
        self.pipeline_hand.stop()
        

    def setParams(self, params):
        return True

    def update_gripper_state(self):

        self.gripper = self.g.get_node('gripper')
        gr_state = self.arm.get_gripper_state()            
        self.gripper.attrs['gripper_finger_distance'].value = 1 - gr_state
        if not self.moving:
            self.g.update_node (self.gripper)
        else:
            self.moving = False

        

    def update_arm_pose(self):
        new_pos = None
        arm_pose = self.arm.get_pose()
        joints = self.arm.get_joints()

        world = self.g.get_node('arm')

        if self.gripper is not None and world is not None:
            # print ("arm reported: ", arm_pose[3:])
            int_rot = R.from_euler('xyz', arm_pose[3:], degrees=True).as_euler('XYZ')
            # print (arm_pose[3:], "--", np.degrees(int_rot))
            arm_rot = int_rot

            new_pos = [self.gripper.id, np.multiply(arm_pose[:3], 1000), arm_rot]
            self.rt.insert_or_assign_edge_RT(world, *new_pos)

            world.attrs['robot_local_angular_velocity'].value = joints
            # print (joints)

            self.g.update_node(world)

    def set_occupied (self, occupied):
        self.occupied = occupied
        self.gripper = self.g.get_node('gripper')
        self.gripper.attrs['robot_occupied'].value = occupied
        self.g.update_node(self.gripper)


    def move_arm_to (self, target_position):

        self.set_occupied (True)

        target_xyz = np.multiply(target_position[:3], 0.001)
        # target_rpy = np.degrees (target_position[3:])

        # TODO revisar luego de cambiar rotaciones
        # target_rpy[0],target_rpy[1] = target_rpy[1],target_rpy[0] 
        target_rpy = R.from_euler('XYZ', target_position[3:]).as_euler('xyz')
        target_rpy = np.degrees (target_rpy)

        print (target_xyz, target_rpy)

        self.arm.cartesian_move_to (*target_xyz, *target_rpy)

        self.set_occupied (False)

    def move_gripper_to (self, dest_distance):
        self.set_occupied (True)
        inv_dest = 1 - dest_distance
        self.arm.move_gripper_speed_dest(inv_dest)
        self.set_occupied (False)

    def stop_moving(self):
        self.arm.stop_movement ()
        # self.set_occupied (False)

    def update_arm_info (self):
        while(True):
            time.sleep(0.05)
            self.update_arm_pose()
            self.update_gripper_state()

    def arm_mover_thread (self):
        while (True):
            if not np.array_equal(self.target_position, self.last_target_position):
                print ("Sending moving command to", self.target_position)
                self.move_arm_to (self.target_position)
                self.last_target_position = np.copy(self.target_position)

            if self.gripper_target_position != self.gripper_last_target_position:
                print ("Sending Gripper command", self.gripper_target_position)
                self.move_gripper_to (self.gripper_target_position)
                self.gripper_last_target_position = self.gripper_target_position

    @QtCore.Slot()
    def compute(self):

        # self.update_arm_pose()
        # self.update_gripper_state()

        frames = self.pipeline_hand.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            print ('No hand frame')
            return True

        
         # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # images = np.hstack((color_image, depth_colormap))
        # images = np.hstack((color_image_h, depth_colormap_h))

        # # Show images
        # cv2.namedWindow('RealSense hand', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense hand', images)
        # cv2.waitKey(1)

        # Suboptimal, should treat them independently
        if color_image is not None and depth_image is not None: 
            
            before = time.time()
            depth = depth_image.tobytes()
            depth = np.frombuffer(depth, dtype=np.uint8)
            depth = depth.reshape((480, 640, 2)) 

            self.camera_node = self.g.get_node('hand_camera')

            if self.camera_node is not None:
                self.camera_node.attrs['cam_rgb'].value = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                self.camera_node.attrs['cam_rgb_height'].value = color_image.shape[0]
                self.camera_node.attrs['cam_rgb_width'].value  = color_image.shape[1]
                self.camera_node.attrs['cam_rgb_depth'].value  = color_image.shape[2]
                self.camera_node.attrs['cam_rgb_focalx'].value = float(color_intrin.fx)
                self.camera_node.attrs['cam_rgb_focaly'].value = float(color_intrin.fy)

                self.camera_node.attrs['cam_depth'].value = depth
                self.camera_node.attrs['cam_depth_height'].value = depth.shape[0]
                self.camera_node.attrs['cam_depth_width'].value  = depth.shape[1]
                # self.camera_node.attrs['cam_ed_depth'].value = color.shape[2]
                self.camera_node.attrs['cam_depth_focalx'].value = float(depth_intrin.fx)
                self.camera_node.attrs['cam_depth_focaly'].value = float(depth_intrin.fy)

                
                self.g.update_node(self.camera_node)
        '''
        if not np.array_equal(self.target_position, self.last_target_position):
            print ("Sending moving command to", self.target_position)
            self.move_arm_to (self.target_position)
            self.last_target_position = np.copy(self.target_position)

        if self.gripper_target_position != self.gripper_last_target_position:
            print ("Sending Gripper command", self.gripper_target_position)
            self.move_gripper_to (self.gripper_target_position)
            self.gripper_last_target_position = self.gripper_target_position
        '''
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        # console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

        if id == self.GRIPPER_ID and 'target' in attribute_names:
            self.moving = True
            updated_node = self.g.get_node(id)
            self.target_position  = updated_node.attrs['target'].value
            print ("Received target position", self.target_position)
            # if self.occupied:
            #     self.stop_moving ()
            # self.move_arm_to (target_position)

        if id == self.GRIPPER_ID and 'gripper_target_finger_distance' in attribute_names:
            updated_node = self.g.get_node(id)
            target_distance  = updated_node.attrs['gripper_target_finger_distance'].value
            self.gripper_target_position = target_distance
            print ("Received target finger distance", target_distance)

        if id == self.GRIPPER_ID and 'robot_occupied' in attribute_names: 
            updated_node = self.g.get_node(id)
            occupied  = updated_node.attrs['robot_occupied'].value
            if not occupied:
                self.stop_moving ()

    def update_node(self, id: int, type: str):
        # console.print(f"UPDATE NODE: {id} {type}", style='green')
        pass

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
