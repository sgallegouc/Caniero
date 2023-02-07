#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
import cv2
import apriltag
import time
import numpy as np
import random
from numpy import linalg as LA
import threading, queue
import interfaces
import matplotlib.pyplot as plt
from collections import deque
from scipy import stats
from loguru import logger

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            # image
            self.image = []
            self.depth = []
            self.camera_name = "camera_arm"

            # apriltags
            self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))

            # get current position
            self.tool_initial_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            self.tool_initial_pose_np = np.array([self.tool_initial_pose.x, self.tool_initial_pose.y, self.tool_initial_pose.z])
            print("Initial pose:", self.tool_initial_pose)
            self.PICK_UP = True
            self.PUT_DOWN = False
            self.pick_up_queue = queue.Queue()
            self.put_down_queue = queue.Queue()
            self.put_down_thread = None
            self.pick_up_thread = None
            self.top_left = []
            self.buttom_right = []
            self.target_global = None
            self.side = 0
            self.last_tag_id = -1
            np.random.seed()
            self.tags = []

            # time series
            plt.ion()
            self.visible = 120
            self.dopening = deque(np.zeros(self.visible), self.visible)
            self.ddistance = deque(np.zeros(self.visible), self.visible)
            self.dforce_left = deque(np.zeros(self.visible), self.visible)
            self.dforce_right = deque(np.zeros(self.visible), self.visible)
            self.dforce_left_tip = deque(np.zeros(self.visible), self.visible)
            self.dforce_right_tip = deque(np.zeros(self.visible), self.visible)
            self.dx = deque(np.zeros(self.visible), self.visible)
            self.data_length = np.linspace(0, 121, num=120)
            ## plt
            self.fig = plt.figure(figsize=(8, 3))
            self.ah1 = self.fig.add_subplot()
            plt.margins(x=0.001)
            self.ah1.set_ylabel("Gripper", fontsize=14)
            self.opening, = self.ah1.plot(self.dx, self.dopening, color='green', label="Closing (x10)", linewidth=1.0)
            self.distance, = self.ah1.plot(self.dx, self.ddistance, color='blue', label="Distance (x10)", linewidth=1.0)
            self.force_left, = self.ah1.plot(self.dx, self.dforce_left, color='red', label="L-Force", linewidth=1.0)
            self.force_right, = self.ah1.plot(self.dx, self.dforce_right, color='magenta', label="R-Force", linewidth=1.0)
            self.force_left_tip, = self.ah1.plot(self.dx, self.dforce_left_tip, color='orange', label="LT-Force", linewidth=1.0)
            self.force_right_tip, = self.ah1.plot(self.dx, self.dforce_right_tip, color='yellow', label="RT-Force",
                                              linewidth=1.0)
            self.ah1.legend(loc="upper right", fontsize=12, fancybox=True, framealpha=0.5)
            self.x_data = 0

            self.timer.timeout.connect(self.compute)
            #self.timer.setSingleShot(True)
            self.timer.start(self.Period)
            
            self.constants = {"camera_y_offset": 42.5,
                              "tag_size": 0.04,             # meters
                              "camera_z_offset": 134,
                              "block_side": 60,
                              "extended_block_side": 90,
                              "max_iter_free_spot": 30000,
                              "min_match_percentage_free_spot": 0.97,
                              "threshold_for_table_distance": 0.01,
                              "max_error_for_tip_positioning": 10,
                              "final_tip_position_over_table_before_releasing": 5,
                              "max_X_tip_position": 300,    # referred to base CS
                              "max_Y_tip_position": 100,
                              "max_Z_tip_position": 500,
                              "min_X_tip_position": -300,
                              "min_Y_tip_position": -550,
                              "min_Z_tip_position": 0}


    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    @logger.catch
    def compute(self):
        color, depth, all = self.read_camera()
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        color, self.tags = self.compute_april_tags(color, depth)
        self.gripper = self.kinovaarm_proxy.getGripperState()
        self.draw_gripper_series(self.gripper)
        # print("dist", int(self.gripper.distance),
        #       "close", int(self.gripper.opening),
        #       "rforce", int(self.gripper.rforce),
        #       "rtip", int(self.gripper.rtipforce),
        #       "lforce", int(self.gripper.lforce),
        #       "ltip", int(self.gripper.ltipforce))  # mm
        if self.PICK_UP:
            if self.pick_up_thread == None:
                visible_tags = [t.tag_id for t in self.tags]
                # remove tag_id from las block moved
                try:
                    visible_tags.remove(self.last_tag_id)
                except:
                    pass
                target_cube = random.choice(visible_tags)
                self.pick_up_thread = threading.Thread(name='pick_up', target=self.pick_up, args=(target_cube, all.image))
                self.pick_up_thread.start()
            else:
                try:
                    state = self.pick_up_queue.get_nowait()
                    print(state)
                    if "Finish" in state:
                        self.PICK_UP = False
                        self.PUT_DOWN = True
                        self.pick_up_thread = None
                        self.last_tag_id = target_cube
                        print("Pick-up DONE")
                    if "Fail" in state:
                        self.pick_up_thread = None
                        self.last_tag_id = target_cube
                        print("Pick-up FAIL")
                except:
                    pass

        if self.PUT_DOWN:
            if self.put_down_thread == None:
                self.put_down_thread = threading.Thread(name='put_down', target=self.put_down, args=(color, depth, all.image))
                self.put_down_thread.start()
            else:
                try:
                    state = self.put_down_queue.get_nowait()
                    print(state)
                    if "Finish" in state:
                        self.PUT_DOWN = False
                        self.put_down_thread = None
                        self.PICK_UP = True
                        self.target_global = None
                        print("Put down DONE")
                except:
                    pass

        self.draw_image(color, all)

    # ===================================================================
    # locate x and do ballistic approach to x proximity, then grasp and go back to initial pos
    def pick_up(self, x, image):
        #####################################################################################
        # STEP A: locate object X and compute target coordinates to approach it
        #####################################################################################
        tx = [t for t in self.tags if t.tag_id == x]
        if not tx:
            self.pick_up_queue.put("PICK_UP: Finish without finding target cube in tags list")
            return False
            
        tx_pose = self.detector.detection_pose( tx[0], [image.focalx, image.focaly, image.width // 2,
                                                image.height // 2], tag_size=self.constants["tag_size"],
                                                z_sign=1)
        tr = tx_pose[0][:, 3][:-1]*1000  # first three elements of last column, aka translation

        # TODO: update block size here, so the code does not depend on this a priori constant

        #####################################################################################
        # STEP B: send arm to target location above selected block
        #####################################################################################
        self.pick_up_queue.put("PICK_UP: Target position in camera ref system: " + str(tr))
        current_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
        target = interfaces.RoboCompKinovaArm.TPose()
        # transform target from camera reference ststem to tip reference system
        target.x = current_pose.x - tr[0]
        target.y = current_pose.y + tr[1] - self.constants["camera_y_offset"]   # plus distance from camera to top along Y
        target.z = current_pose.z - tr[2] + self.constants["camera_z_offset"]   # distance from camera to tip along Z
        self.pick_up_queue.put("PICK_UP: Tip sent to target")
        self.kinovaarm_proxy.setCenterOfTool(target, interfaces.RoboCompKinovaArm.ArmJoints.base)
        target_pose = np.array([target.x, target.y, target.z])

        # wait for the arm to complete the movement
        dist = sys.float_info.max
        while dist > self.constants["max_error_for_tip_positioning"]:
            pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - target_pose)

        #####################################################################################
        # STEP C: look again and move arm down to grasping position
        #####################################################################################
        # TODO: Check if there is room for the GRIPPER
        self.pick_up_queue.put("PICK_UP: Initiating grasping")
        pre_grip_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
        tx = [t for t in self.tags if t.tag_id == x]
        if tx:
            tx_pose = self.detector.detection_pose(tx[0], [image.focalx, image.focaly, image.width//2,
                                                   image.height//2], tag_size=self.constants["tag_size"],
                                                   z_sign=1)
            tr = tx_pose[0][:, 3][:-1] * 1000  # first three elements of last column, aka translation
            pre_grip_pose.x -= tr[0]
            pre_grip_pose.y += tr[1] - self.constants["camera_y_offset"]
            self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
            time.sleep(0.5)

        # TODO correct orientation wrt Z axis from tag reading

        iter = 0
        # If NO readings yet from the gripper distance sensor
        while self.gripper.distance < 0 and iter < 8:
            pre_grip_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            pre_grip_pose.x += random.randint(-10, 10)
            pre_grip_pose.y += random.randint(-10, 10)
            pre_grip_pose.z -= random.randint(1, 10)
            self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
            time.sleep(0.1)
            iter += 1
            print("RE-ADJUSTING")
        if self.gripper.distance > 0:
            # TODO: center now again on the cube
            while self.gripper.distance > 20:
                    pre_grip_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
                    pre_grip_pose.z -= 10
                    self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
                    #print("dist while lowering", int(self.gripper.distance))
                    time.sleep(0.07)

        # could not approach safely. Go for a blind move
    # else:
        #     pre_grip_pose.z -= 100
        #     self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
        #     time.sleep(0.2)


        #####################################################################################
        # STEP D: Grasp object (the data coming from the gripper's sensors should match a know pattern)
        #####################################################################################
        self.kinovaarm_proxy.closeGripper()
        init_time = time.time()
        while self.gripper.opening < 200 and self.gripper.rforce < 1000 and (time.time() - init_time) < 2:
            # Check if finger tips have hit the table
            if abs(self.gripper.ltipforce) > 5 or abs(self.gripper.rtipforce) > 5:
                print("SHIT I hit a block")
                self.kinovaarm_proxy.setCenterOfTool(self.tool_initial_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
                dist = sys.float_info.max
                while dist > self.constants["max_error_for_tip_positioning"]:  # mm
                    pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
                    dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - self.tool_initial_pose_np)
                self.pick_up_queue.put("PICK_UP: Fail while grasping block " + str(x) + " Hit something. Sent to initial position")
                return False
            # Check if the gripper is closing too much > 400
            if self.gripper.opening > 400:
                print("SHIT I missed the block")
                self.kinovaarm_proxy.setCenterOfTool(self.tool_initial_pose,
                                                     interfaces.RoboCompKinovaArm.ArmJoints.base)
                dist = sys.float_info.max
                while dist > self.constants["max_error_for_tip_positioning"]:  # mm
                    pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
                    dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - self.tool_initial_pose_np)
                self.pick_up_queue.put("PICK_UP: Fail while grasping block " + str(x) + " Missed the block. Sent to initial position")
                return False
                print("dist", int(self.gripper.distance),
                      "close", int(self.gripper.opening),
                      "rforce", int(self.gripper.rforce),
                      "rtip", int(self.gripper.rtipforce),
                      "lforce", int(self.gripper.lforce),
                      "ltip", int(self.gripper.ltipforce))  # mm
            time.sleep(0.1)


        #####################################################################################
        # STEP E: Move to initial position
        #####################################################################################
        self.kinovaarm_proxy.setCenterOfTool(self.tool_initial_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
        self.pick_up_queue.put("PICK_UP: Tip sent to initial position")
        dist = sys.float_info.max
        while dist > self.constants["max_error_for_tip_positioning"]:  # mm
            pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - self.tool_initial_pose_np)

        self.pick_up_queue.put("PICK_UP: Finish")
        return True         

    def put_down(self, color, depth, image):
        #####################################################################################
        # STEP A: Search for a free spot for the current object
        #####################################################################################
        # compute side of box in pixels from current location
        self.put_down_queue.put("PUT_DOWN: Searching for free spot")
        current_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
        z = current_pose.z + self.constants["camera_z_offset"]    # camera distance from gripper tip along Z
        side = int(image.focalx * self.constants["extended_block_side"] / z)  # Project. Result in pixels
        print("side", side, "focal", image.focalx, "cam distance", int(z))

        # compute the color of the table (blue) to choose a free spot to drop the block
        #hist = cv2.calcHist([color], [2], None, [256], [0, 256])
        #table_color = np.argmax(hist)
        #print("table_color", table_color)

        # choose a random rectangle on the table (check with distance from the camera)
        result = False
        counter = 0
        hits = 0
        roi = []
        target = interfaces.RoboCompKinovaArm.TPose()
        target.x = sys.float_info.max
        while (not result or not self.in_limits(target)) and counter < self.constants["max_iter_free_spot"]:
            # image is treated as np array, with rows first
            row = random.randint(side//2, image.height-side//2)
            col = random.randint(side//2, image.width-side//2)
            roi_depth = depth[row-side//2:row+side//2, col-side//2:col+side//2]
            table_distance_from_tip = stats.mode(roi_depth, axis=None)[0]
            # check that all points are close to the table color
            # blue = np.array(roi[:, :, 2])  # blue channel
            # hits = np.count_nonzero(abs(blue - table_color) < 5)

            # check that all points belong to the table
            hits = np.count_nonzero(abs(roi_depth - table_distance_from_tip)
                                    < self.constants["threshold_for_table_distance"])
            counter += 1
            result = hits >= (roi_depth.size * self.constants["min_match_percentage_free_spot"])

            # compute target coordinates. x, y, z in camera CS: x+ right, y+ up, z+ outwards
            target.x = (image.width//2 - col) * z / image.focalx
            target.y = (image.height//2 - row) * z / image.focaly

            # transform to tip coordinate system: x+ right, y+ backwards, z+ up
            target.y = -target.y - self.constants["camera_y_offset"]  # distance from camera to tip along Y
            target.z = -current_pose.z

            # transform to base coordinate system
            target.x += current_pose.x
            target.y += current_pose.y
            target.z += current_pose.z

            #print("Hits", hits, "from total size", roi_depth.size, "in ", counter, "iters", "In bounds:", self.in_limits(target))

        if counter == self.constants["max_iter_free_spot"]:
            self.put_down_queue.put("PUT_DOWN: Could NOT find a free spot")
            return False

        #rgb = color.copy()
        #rgb = cv2.rectangle(rgb, (col-side // 2, row-side // 2), (col+side//2, row+side//2), (255, 0, 0), 6)
        #rgb = cv2.rectangle(rgb, (side//2, side//2), (image.width-side//2, image.height-side // 2), (255, 0, 0), 6)
        #cv2.imshow("roi", rgb)

        print("Number of hits", hits, "from total size", roi_depth.size, "in ", counter, "iters")
        print("Center in image row,col: ", row, col)
        print("target:", int(target.x), int(target.y), int(target.z))
        self.target_global = target     # for drawing
        self.put_down_queue.put("PUT_DOWN: free SPOT found at " + str([target.x, target.y, target.z]))

        #####################################################################################
        # STEP B: Move to target spot
        #####################################################################################
        # B.1 move to target position at same Z level
        target.z = current_pose.z
        self.put_down_queue.put("PUT_DOWN: Tip sent to target")
        self.kinovaarm_proxy.setCenterOfTool(target, interfaces.RoboCompKinovaArm.ArmJoints.base)
        target_np = np.array([target.x, target.y, target.z])

        # wait for the arm to complete the movement
        dist = sys.float_info.max
        while dist > self.constants["max_error_for_tip_positioning"]:
            pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - target_np)

        # B.2 move down to free spot position
        target.z = self.constants["final_tip_position_over_table_before_releasing"]
        target_np = np.array([target.x, target.y, target.z])
        self.kinovaarm_proxy.setCenterOfTool(target, interfaces.RoboCompKinovaArm.ArmJoints.base)
        # wait for the arm to complete the movement
        dist = sys.float_info.max
        while dist > self.constants["max_error_for_tip_positioning"]:
            pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - target_np)

        #####################################################################################
        # STEP C: release block
        #####################################################################################
        self.put_down_queue.put("PUT_DOWN: Initiating release")
        self.kinovaarm_proxy.openGripper()
        time.sleep(0.5)

        #####################################################################################
        # STEP D: move back to initial position
        #####################################################################################
        # move upwards a bit so the gripper does not hit the block
        current_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
        current_pose.z -= 100
        self.kinovaarm_proxy.setCenterOfTool(self.tool_initial_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
        time.sleep(0.5)
        # keep moving to initial position
        self.kinovaarm_proxy.setCenterOfTool(self.tool_initial_pose, interfaces.RoboCompKinovaArm.ArmJoints.base)
        self.put_down_queue.put("PUT_DOWN: Tip sent to initial position")
        dist = sys.float_info.max
        while dist > self.constants["max_error_for_tip_positioning"]:  # mm
            pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - self.tool_initial_pose_np)
        self.put_down_queue.put("PUT_DOWN: Finish")

        # TODO: Check if the block is in a correct position
        return True

    def stack(self, x, y):
        pass

    def unstack(self, x, y):
        pass

    #########################################################################################
    def in_limits(self, target):
        if (self.constants["max_X_tip_position"] > target.x > self.constants["min_X_tip_position"])\
                and \
           (self.constants["max_Y_tip_position"] > target.y > self.constants["min_Y_tip_position"]):
            return True
        else:
            return False
    def draw_gripper_series(self, gdata):
        # update data
        # print(int(gripper.opening*100), int(gripper.lforce*10000), int(gripper.rforce*10000), int(gripper.distance * 1000))
        self.dopening.extend([gdata.opening])
        self.ddistance.extend([gdata.distance])
        self.dforce_left.extend([gdata.lforce*10])
        self.dforce_right.extend([gdata.rforce*10])
        self.dforce_left_tip.extend([gdata.ltipforce*10])
        self.dforce_right_tip.extend([gdata.rtipforce*10])
        self.dx.extend([self.x_data])

        # update plot
        self.opening.set_ydata(self.dopening)
        self.opening.set_xdata(self.dx)
        self.distance.set_ydata(self.ddistance)
        self.distance.set_xdata(self.dx)
        self.force_left.set_ydata(self.dforce_left)
        self.force_left.set_xdata(self.dx)
        self.force_right.set_ydata(self.dforce_right)
        self.force_right.set_xdata(self.dx)
        self.force_left_tip.set_ydata(self.dforce_left_tip)
        self.force_left_tip.set_xdata(self.dx)
        self.force_right_tip.set_ydata(self.dforce_right_tip)
        self.force_right_tip.set_xdata(self.dx)

        # set axes
        self.ah1.set_ylim(-10, 1000)
        self.ah1.set_xlim(self.x_data-self.visible, self.x_data)

        # control speed of moving time-series
        self.x_data += 1

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    def read_camera(self):
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        color = np.frombuffer(all.image.image, np.uint8).reshape(all.image.height, all.image.width, all.image.depth)
        depth = np.frombuffer(all.depth.depth, np.float32).reshape(all.depth.height, all.depth.width)
        return color, depth, all
    def draw_image(self, color, all):
        if self.target_global:
            self.target_global.z = 0
            #print("self.target: ", self.target_global.x, self.target_global.y, self.target_global.z)
            # we need to transform target to the camera's RS since it has moved
            current_pose = self.kinovaarm_proxy.getCenterOfTool(interfaces.RoboCompKinovaArm.ArmJoints.base)
            target = interfaces.RoboCompKinovaArm.TPose()

            # transform target from base CS to tip CS
            target.x = self.target_global.x - current_pose.x
            target.y = self.target_global.y - current_pose.y
            target.z = self.target_global.z - current_pose.z
            #print("base to tip: ", target.x, target.y, target.z)

            # transform target from tip CS to camera CS
            target.y = -target.y - self.constants["camera_y_offset"]
            target.z = -target.z + self.constants["camera_z_offset"]
            #print("tip to camera: ", target.x, target.y, target.z)

            # compute size of rectangle from current arm position
            side = int(all.image.focalx * self.constants["block_side"] / target.z)  # 60 is the  size of the block in mm. Result in pixels

            # project on camera
            cx = int(-all.image.focalx * target.x / target.z) + all.image.width//2
            cy = int(-all.image.focaly * target.y / target.z) + all.image.height//2
            #print(side, cx, cy, target.z)
            color = cv2.rectangle(color, (cx - side // 2, cy - side // 2), (cx + side // 2, cy + side // 2), (255, 0, 0), 6)
            cv2.drawMarker(color, (all.image.width//2, all.image.height//2), (0, 255, 0), cv2.MARKER_CROSS, 300, 1);
        cv2.imshow("Gen3", color)
        cv2.waitKey(1)
    def compute_april_tags(self, color, depth):
        tags = self.detector.detect(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        if len(tags) > 0:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(color, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                    cv2.putText(color, str(tag.tag_id),
                               org = (tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                               fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
                    cv2.rectangle(color, tuple(tag.center.astype(int) - 1), tuple(tag.center.astype(int) + 1), (255, 0, 255))
        else:
            #print("Compute_april_tags: No tags detected")
            pass

        return color, tags

    #########################################################################################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm_proxy.closeGripper(...)
    # self.kinovaarm_proxy.getCenterOfTool(...)
    # self.kinovaarm_proxy.openGripper(...)
    # self.kinovaarm_proxy.setCenterOfTool(...)
    # self.kinovaarm_proxy.setGripper(...)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


