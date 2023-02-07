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

import os
import cv2
import math
import time
import random
import apriltag
import interfaces
import numpy as np
import threading, queue
from scipy import stats
from loguru import logger
from genericworker import *
from collections import deque
from numpy import linalg as LA
import matplotlib.pyplot as plt
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.base = interfaces.RoboCompKinovaArm.ArmJoints.base

            # image
            self.image = []
            self.depth = []
            self.camera_name = "camera_arm"

            # apriltags
            self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))

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

            # plt
            self.fig = plt.figure(figsize=(8, 3))
            self.ah1 = self.fig.add_subplot()
            plt.margins(x=0.001)
            self.ah1.set_ylabel("Gripper", fontsize=14)
            self.opening, = self.ah1.plot(self.dx, self.dopening, color='green', label="Closing (x10)", linewidth=1.0)
            self.distance, = self.ah1.plot(self.dx, self.ddistance, color='blue', label="Distance (x10)", linewidth=1.0)
            self.force_left, = self.ah1.plot(self.dx, self.dforce_left, color='red', label="L-Force", linewidth=1.0)
            self.force_right, = self.ah1.plot(self.dx, self.dforce_right, color='magenta', label="R-Force", linewidth=1.0)
            self.force_left_tip, = self.ah1.plot(self.dx, self.dforce_left_tip, color='orange', label="LT-Force", linewidth=1.0)
            self.force_right_tip, = self.ah1.plot(self.dx, self.dforce_right_tip, color='yellow', label="RT-Force", linewidth=1.0)
            self.ah1.legend(loc="upper right", fontsize=12, fancybox=True, framealpha=0.5)
            self.x_data = 0

            plt.show()

            # get current position
            self.osberving_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
            self.osberving_pose_np = np.array([self.osberving_pose.x, self.osberving_pose.y, self.osberving_pose.z])
            print("Observing pose:", self.osberving_pose)
            self.target_global = None

            self.working_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
            self.working_pose.x = 27
            self.working_pose.y = -500
            self.working_pose.z = 250
            self.working_pose.rx = -90 * math.pi / 180
            self.working_pose.ry = 0 * math.pi / 180
            self.working_pose.rz = -90 * math.pi / 180

            # TODO: Definir y aplicar rotación para poner la mano en vertical (se ven los cubos desde arriba)

            self.working_pose_np = np.array([self.working_pose.x, self.working_pose.y, self.working_pose.z])

            # Planning 
            self.step = 0
            self.end = False
            self.endState = []
            self.plan = []

            # Dictionary to organize actions
            self.actions = {
                "pick-up" : {
                    "do_action" : False,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.pick_up,
                },
                "put-down" : {
                    "do_action" : False,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.put_down
                },
                "stack": {
                    "do_action" : False,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.stack
                },
                "unstack": {
                    "do_action" : False,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.unstack
                },
            }

            self.constants = {
                "camera_y_offset": 42.5, # mm?
                "tag_size": 0.04, # m
                "camera_z_offset": 134, # mm?
                "block_side": 60,
                "extended_block_side": 90,
                "max_iter_free_spot": 30000, # no.
                "min_match_percentage_free_spot": 0.97,
                "threshold_for_table_distance": 0.01,
                "max_error_for_tip_positioning": 10, # mm
                "final_tip_position_over_table_before_releasing": 5,
                "max_X_tip_position": 300,    # referred to base CS
                "max_Y_tip_position": 100,
                # "max_Z_tip_position": 500,
                "min_X_tip_position": -300,
                "min_Y_tip_position": -550,
                # "min_Z_tip_position": 0,
                "tag_center_distance_threshold": 30, #pixels
                "init_state_file": "init_state.pdll",
                "init_state_path": f"/home/robocomp/robocomp/components/manipulation_kinova_gen3/components/controllerDani/init_state.pdll",
                "plan_file": "/home/robocomp/software/fast-downward-20.06/sas_plan",
                "block_threshold": 10
            }

            # set of candidate positions for blocks on the table
            # a = RoboCompKinovaArm.TPose([74, 450, 30])

            self.timer.timeout.connect(self.compute)
            # self.timer.setSingleShot(True)
            self.timer.start(self.Period)

            self.begin_plan = False

            self.ui.generar.clicked.connect(self.create_final_state)

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
        self.draw_image(color, depth, all)

        if self.end:
            print("PLAN ENDED")
        else:
            if self.begin_plan:
                self.begin_plan = False
                tag_list = self.create_initial_state()
                # for rule in self.initState:
                #     print(rule)
                self.save_to_file(self.initState, self.endState, tag_list)
                self.exec_planner()
                self.load_plan()
                print(self.plan)

                # TODO: Corregir posición del codo
                self.kinovaarm_proxy.setCenterOfTool(self.working_pose, self.base)
                self.wait_to_complete_movement(self.working_pose_np)

                current_step = self.plan[self.step]
                current_action = self.actions[current_step[0]]
                current_action["do_action"] = True
                # self.actions[self.plan[0][0]]["do_action"] = True
            else:
                if self.plan != []:
                    current_step = self.plan[self.step]
                    # print(current_step)
                    current_action = self.actions[current_step[0]]
                    params = current_step[1] if len(current_step) > 1 else None
                    args_list = []
                    if current_action["do_action"]:
                        if current_action["thread"] == None:
                            if current_step[0] == "pick-up":
                                args_list = [params, all.image]
                            elif current_step[0] == "put-down":
                                args_list = [color, depth, all.image]
                            elif current_step[0] == "stack":
                                args_list = [color, depth, all.image, params[1]]
                            elif current_step[0] == "unstack":
                                args_list = [params[0], all.image]
                            current_action["thread"] = threading.Thread(name=current_step[0], target=current_action["func"], args=tuple(args_list))
                            current_action["thread"].start()
                        else:
                            try:
                                state = current_action["queue"].get_nowait()
                                print(state)
                                if "Finish" in state:
                                    current_action["do_action"] = False
                                    current_action["thread"] = None
                                    # print(f"{current_step[0]} DONE")
                                    self.step += 1
                                    if self.step == len(self.plan):
                                        self.end = True
                                    else:
                                        next_step = self.plan[self.step]
                                        next_action = self.actions[next_step[0]]
                                        next_action["do_action"] = True
                            except:
                                pass

    # =======================================================================================
    # State related functions 
    # =======================================================================================
    def create_final_state(self):
        self.endState = ["  (and(handempty)"]
        
        b1 = (1, self.ui.cubo1.x(), self.ui.cubo1.y())
        b2 = (2, self.ui.cubo2.x(), self.ui.cubo2.y())
        b3 = (3, self.ui.cubo3.x(), self.ui.cubo3.y())
        b4 = (4, self.ui.cubo4.x(), self.ui.cubo4.y())
        b5 = (5, self.ui.cubo5.x(), self.ui.cubo5.y())
        b6 = (6, self.ui.cubo6.x(), self.ui.cubo6.y())
        cubos = [b1, b2, b3, b4, b5, b6]
        cubos.sort(key=lambda x: x[2], reverse=True)
        
        cubo_mesa = cubos[0]
        cubos_aux = []
        for cubo in cubos:
            if abs(cubo_mesa[2] - cubo[2]) < self.constants["block_threshold"]:
                self.endState.append(f"     (ontable {cubo[0]})")
                cubos_aux.append(cubo)
            else:
                for c_aux in cubos_aux:
                    if abs(cubo[1] - c_aux[1]) < self.constants["block_threshold"] and cubo[0] != c_aux[0]:
                        self.endState.append(f"     (on {cubo[0]} {c_aux[0]})")
                        cubos_aux.remove(c_aux)
                        cubos_aux.append(cubo)
        for cubo in cubos_aux:
            self.endState.append(f"     (clear {cubo[0]})")
        self.begin_plan = True
        self.end = False
        self.step = 0
        print(self.endState)
        
    def create_initial_state(self):
        self.initState = ["  (handempty)"]
        
        tags = []
        for tag in self.tags:
            if self.is_horizontal(tag):
                self.initState.append(f"  (clear {tag.tag_id})")
            else:
                tags.append(tag)
        ret_tags = tags
        tags.sort(key=lambda x: x.center[1], reverse=True)
        tag_base = tags[0]
        tags_aux = []
        for tag in tags:
            if abs(tag_base.center[1] - tag.center[1]) < self.constants["tag_center_distance_threshold"]:
                self.initState.append(f"  (ontable {tag.tag_id})")
                tags_aux.append(tag)
            else:
                for tag_aux in tags_aux:
                    if abs(tag.center[0] - tag_aux.center[0]) < self.constants["tag_center_distance_threshold"] and tag.tag_id != tag_aux.tag_id:
                        self.initState.append(f"  (on {tag.tag_id} {tag_aux.tag_id})")
                        tags_aux.remove(tag_aux)
                        tags_aux.append(tag)
        return ret_tags
    
    def is_horizontal(self, tag):
        up_line = LA.norm(np.array(tag.corners[3]) - np.array(tag.corners[2]))
        down_line = LA.norm(np.array(tag.corners[1]) - np.array(tag.corners[0]))
        return down_line > up_line
    
    def save_to_file(self, init_state, end_state, tag_list):
        tag_list.sort()
        blocks = ""
        for tag in tag_list:
            blocks += str(tag.tag_id) + " "

        for r in init_state:
            print(r)
        for r in end_state:
            print(r)
        lines = [
            "(define (problem BLOCKS-{}-1)".format(len(tag_list)),
            "(:domain blocks)",
            "(:objects {}- block)".format(blocks),
            "(:init",
            *init_state,
            ")",
            "(:goal",
            *end_state, 
            "   )",
            ")",
            ")"
        ]
        with open(self.constants["init_state_file"], 'w+') as f:
            f.write('\n'.join(lines))
    
    def exec_planner(self):
        os.system("cd /home/robocomp/software/fast-downward-20.06/ && ls && \
        ./fast-downward.py --alias lama-first /home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/domain.pddl {}".format(self.constants["init_state_path"]))
    
    def load_plan(self):
        with open(self.constants["plan_file"], 'r+') as f:
            lines = f.readlines()
            for line in lines:
                if ";" not in line:
                    action, *rest = line[1:-2].split()
                    self.plan.append([action, list(map(lambda x: int(x), rest))])

    # =======================================================================================
    # Functions for the different actions actions
    # =======================================================================================
    def pick_up(self, x, image, unstack=False):
        action_queue = self.actions["pick-up"]["queue"] if not unstack else self.actions["unstack"]["queue"]
        action = "PICK_UP" if not unstack else "UNSTACK"
        #####################################################################################
        # STEP A: locate object X and compute target coordinates to approach it
        #####################################################################################
        tx = [t for t in self.tags if t.tag_id == x]
        if not tx:
            action_queue.put(action + ": Finish without finding target cube in tags list")
            return False

        tx_pose = self.detector.detection_pose(tx[0], [image.focalx, image.focaly, image.width // 2, image.height // 2], 
                                                tag_size=self.constants["tag_size"], z_sign=1)
        tr = tx_pose[0][:, 3][:-1]*1000  # first three elements of last column, aka translation
        
        # TODO: update block size here, so the code does not depend on this a priori constant

        #####################################################################################
        # STEP B: send arm to target location above selected block
        #####################################################################################
        action_queue.put(action + ": Target position in camera ref system: " + str(tr))
        current_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
        target = interfaces.RoboCompKinovaArm.TPose()
        # transform target from camera reference ststem to tip reference system
        target.x = current_pose.x - tr[0]
        target.y = current_pose.y + tr[1] - self.constants["camera_y_offset"]   # plus distance from camera to top along Y
        target.z = current_pose.z - tr[2] + self.constants["camera_z_offset"]   # distance from camera to tip along Z
        target.rx = -90 * math.pi / 180
        target.ry = 0 * math.pi / 180
        target.rz = -90 * math.pi / 180
        action_queue.put(action + ": Tip sent to target")
        self.kinovaarm_proxy.setCenterOfTool(target, self.base)
        target_pose = np.array([target.x, target.y, target.z])
        self.wait_to_complete_movement(target_pose)

        #####################################################################################
        # STEP C: look again and move arm down to grasping position
        #####################################################################################
        
        # TODO: Check if there is room for the GRIPPER
        
        action_queue.put(action + ": Initiating grasping")
        pre_grip_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
        tx = [t for t in self.tags if t.tag_id == x]
        if tx:
            tx_pose = self.detector.detection_pose(tx[0], [image.focalx, image.focaly, image.width // 2, image.height // 2],
                                                    tag_size=self.constants["tag_size"], z_sign=1)
            tr = tx_pose[0][:, 3][:-1] * 1000  # first three elements of last column, aka translation
            pre_grip_pose.x -= tr[0]
            pre_grip_pose.y += tr[1] - self.constants["camera_y_offset"]
            self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, self.base)
            time.sleep(0.5)

        # TODO correct orientation wrt Z axis from tag reading

        iter = 0
        # If NO readings yet from the gripper distance sensor
        while self.gripper.distance < 0 and iter < 8:
            pre_grip_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
            pre_grip_pose.x += random.randint(-10, 10)
            pre_grip_pose.y += random.randint(-10, 10)
            pre_grip_pose.z -= random.randint(1, 10)
            self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, self.base)
            time.sleep(0.1)
            iter += 1
            print("RE-ADJUSTING")

        if self.gripper.distance > 0:

            # TODO: center now again on the cube

            while self.gripper.distance > 20:
                    pre_grip_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
                    pre_grip_pose.z -= 10
                    self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, self.base)
                    # print("dist while lowering", int(self.gripper.distance))
                    time.sleep(0.07)

        # could not approach safely. Go for a blind move
        # else:
        #     pre_grip_pose.z -= 100
        #     self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, self.base)
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
                self.kinovaarm_proxy.setCenterOfTool(self.tool_initial_pose, self.base)
                self.wait_to_complete_movement(self.tool_initial_pose_np)
                action_queue.put(action + ": Fail while grasping block " + str(x) + " Hit something. Sent to initial position")
                return False
            # Check if the gripper is closing too much > 400
            if self.gripper.opening > 400:
                print("SHIT I missed the block")
                self.kinovaarm_proxy.setCenterOfTool(self.tool_initial_pose, self.base)
                self.wait_to_complete_movement(self.tool_initial_pose_np)
                action_queue.put(action + ": Fail while grasping block " + str(x) + " Missed the block. Sent to initial position")
                return False
            time.sleep(0.1)

        #####################################################################################
        # STEP E: Move to initial position
        #####################################################################################
        self.kinovaarm_proxy.setCenterOfTool(self.working_pose, self.base)
        action_queue.put(action + ": Tip sent to initial position")
        self.wait_to_complete_movement(self.working_pose_np)
        action_queue.put(action + ": Finish")
        return True       

    def put_down(self, color, depth, image, stack=0):
        action_queue = self.actions["put-down"]["queue"] if stack == 0 else self.actions["stack"]["queue"]
        action = "PUT_DOWN" if stack == 0 else "STACK" 
        #####################################################################################
        # STEP A: Search for a free spot for the current object
        #####################################################################################
        
        action_queue.put(action + ": Searching for {} spot".format("free" if stack != 0 else "target"))
        target, current_pose, target_z = self.find_free_spot(depth, image) if stack == 0 else self.find_target_spot(depth, image, stack)
        self.target_global = target     # for drawing
        action_queue.put(action + ": {} SPOT found at ".format("free" if stack != 0 else "target") + str([target.x, target.y, target.z]))

        #####################################################################################
        # STEP B: Move to target spot
        #####################################################################################
        # B.1 move to target position at same Z level
        target.z = current_pose.z
        action_queue.put(action + ": Tip sent to target")
        self.kinovaarm_proxy.setCenterOfTool(target, self.base)
        target_np = np.array([target.x, target.y, target.z])
        self.wait_to_complete_movement(target_np)

        # B.2 move down to free spot position
        target.z = target_z
        target_np = np.array([target.x, target.y, target.z])
        self.kinovaarm_proxy.setCenterOfTool(target, self.base)
        self.wait_to_complete_movement(target_np)

        #####################################################################################
        # STEP C: release block
        #####################################################################################
        action_queue.put(action + ": Initiating release")
        self.kinovaarm_proxy.openGripper()
        time.sleep(0.5)

        #####################################################################################
        # STEP D: move back to initial position
        #####################################################################################
        # move upwards a bit so the gripper does not hit the block
        current_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
        current_pose.z -= 100
        self.kinovaarm_proxy.setCenterOfTool(self.working_pose, self.base)
        time.sleep(0.5)
        # keep moving to initial position
        self.kinovaarm_proxy.setCenterOfTool(self.working_pose, self.base)
        action_queue.put(action + ": Tip sent to initial position")
        self.wait_to_complete_movement(self.working_pose_np)
        action_queue.put(action + ": Finish")

        # TODO: Check if the block is in a correct position

        return True

    def stack(self, color, depth, image, target_block):
        self.put_down(color, depth, image, target_block)

    def unstack(self, x, image):
        self.pick_up(x, image, True)

    # =======================================================================================
    # Auxiliar functions
    # =======================================================================================
    def in_limits(self, target):
        return (self.constants["max_X_tip_position"] > target.x > self.constants["min_X_tip_position"]) and \
               (self.constants["max_Y_tip_position"] > target.y > self.constants["min_Y_tip_position"])

    def draw_gripper_series(self, gdata):
        # update data
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
        # self.fig.canvas.flush_events()

        img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        qt_color = QImage(img, img.shape[1], img.shape[0], QImage.Format_RGB888)
        pix_color = QPixmap.fromImage(qt_color).scaled(self.ui.grafic.width(), self.ui.grafic.height())
        self.ui.grafic.setPixmap(pix_color)

    def read_camera(self):
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        color = np.frombuffer(all.image.image, np.uint8).reshape(all.image.height, all.image.width, all.image.depth)
        depth = np.frombuffer(all.depth.depth, np.float32).reshape(all.depth.height, all.depth.width)
        return color, depth, all

    def draw_image(self, color, depth, all):
        if self.target_global:
            self.target_global.z = 0
            # print("self.target: ", self.target_global.x, self.target_global.y, self.target_global.z)
            # we need to transform target to the camera's RS since it has moved
            current_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
            target = interfaces.RoboCompKinovaArm.TPose()

            # transform target from base CS to tip CS
            target.x = self.target_global.x - current_pose.x
            target.y = self.target_global.y - current_pose.y
            target.z = self.target_global.z - current_pose.z
            # print("base to tip: ", target.x, target.y, target.z)

            # transform target from tip CS to camera CS
            target.y = -target.y - self.constants["camera_y_offset"]
            target.z = -target.z + self.constants["camera_z_offset"]
            # print("tip to camera: ", target.x, target.y, target.z)

            # compute size of rectangle from current arm position
            side = int(all.image.focalx * self.constants["block_side"] / target.z)  # 60 is the  size of the block in mm. Result in pixels

            # project on camera
            cx = int(-all.image.focalx * target.x / target.z) + all.image.width // 2
            cy = int(-all.image.focaly * target.y / target.z) + all.image.height // 2
            # print(side, cx, cy, target.z)
            color = cv2.rectangle(color, (cx - side // 2, cy - side // 2), (cx + side // 2, cy + side // 2), (255, 0, 0), 6)
            cv2.drawMarker(color, (all.image.width // 2, all.image.height // 2), (0, 255, 0), cv2.MARKER_CROSS, 300, 1)
        
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        qt_color = QImage(color, 512, 512, QImage.Format_RGB888)
        pix_color = QPixmap.fromImage(qt_color).scaled(self.ui.color.width(), self.ui.color.height())
        self.ui.color.setPixmap(pix_color)

        # depth = np.uint8(depth*255)
        # qt_depth = QImage(depth, 512, 512, QImage.Format_Grayscale8)
        # pix_depth = QPixmap.fromImage(qt_depth).scaled(self.ui.depth.width(), self.ui.depth.height())
        # self.ui.depth.setPixmap(pix_depth)

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
            # print("Compute_april_tags: No tags detected")
            pass
        return color, tags
    
    def wait_to_complete_movement(self, pose_to_reach_np):
        dist = sys.float_info.max
        while dist > self.constants["max_error_for_tip_positioning"]:
            pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - pose_to_reach_np)

    def find_free_spot(self, depth, image):
        # compute side of box in pixels from current location
        current_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
        z = current_pose.z + self.constants["camera_z_offset"]    # camera distance from gripper tip along Z
        side = int(image.focalx * self.constants["extended_block_side"] / z)  # Project. Result in pixels
        print("side", side, "focal", image.focalx, "cam distance", int(z))

        # compute the color of the table (blue) to choose a free spot to drop the block
        # hist = cv2.calcHist([color], [2], None, [256], [0, 256])
        # table_color = np.argmax(hist)
        # print("table_color", table_color)

        # choose a random rectangle on the table (check with distance from the camera)
        result = False
        counter = 0
        hits = 0
        roi = []
        target = interfaces.RoboCompKinovaArm.TPose()
        target.x = sys.float_info.max
        while (not result or not self.in_limits(target)) and counter < self.constants["max_iter_free_spot"]:
            # image is treated as np array, with rows first
            row = random.randint(side // 2, image.height - side // 2)
            col = random.randint(side // 2, image.width - side // 2)
            roi_depth = depth[row-side // 2 : row + side // 2, col - side // 2 : col + side // 2]
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
            target.x = (image.width // 2 - col) * z / image.focalx
            target.y = (image.height // 2 - row) * z / image.focaly

            # transform to tip coordinate system: x+ right, y+ backwards, z+ up
            target.y = - target.y - self.constants["camera_y_offset"]  # distance from camera to tip along Y
            target.z = - current_pose.z

            # transform to base coordinate system
            target.x += current_pose.x
            target.y += current_pose.y
            target.z += current_pose.z
            target.rx = -90 * math.pi / 180
            target.ry = 0 * math.pi / 180
            target.rz = -90 * math.pi / 180

            #print("Hits", hits, "from total size", roi_depth.size, "in ", counter, "iters", "In bounds:", self.in_limits(target))

        if counter == self.constants["max_iter_free_spot"]:
            self.actions["put_down"]["queue"].put("PUT_DOWN: Could NOT find a free spot")
            return False

        # rgb = color.copy()
        # rgb = cv2.rectangle(rgb, (col - side // 2, row - side // 2), (col + side // 2, row + side // 2), (255, 0, 0), 6)
        # rgb = cv2.rectangle(rgb, (side // 2, side // 2), (image.width - side // 2, image.height - side // 2), (255, 0, 0), 6)
        # cv2.imshow("roi", rgb)

        print("Number of hits", hits, "from total size", roi_depth.size, "in ", counter, "iters")
        print("Center in image row,col: ", row, col)
        print("target:", int(target.x), int(target.y), int(target.z))

        return target, current_pose, self.constants["final_tip_position_over_table_before_releasing"]

    def find_target_spot(self, depth, image, target_tag):
        # TODO : Find the target block and return its position
        target = interfaces.RoboCompKinovaArm.TPose()
        current_pose = self.kinovaarm_proxy.getCenterOfTool(self.base)
        z = current_pose.z + self.constants["camera_z_offset"]    # camera distance from gripper tip along Z
        for tag in self.tags:
            if tag.tag_id == target_tag:
                # print(tag)
                row = int(tag.center[1])
                col = int(tag.center[0])
                # compute target coordinates. x, y, z in camera CS: x+ right, y+ up, z+ outwards
                target.x = (image.width // 2 - col) * z / image.focalx
                target.y = (image.height // 2 - row) * z / image.focaly

                # transform to tip coordinate system: x+ right, y+ backwards, z+ up
                target.y = - target.y - self.constants["camera_y_offset"]  # distance from camera to tip along Y
                z_block = depth[row][col]
                target.z = - current_pose.z
                print(z_block)
                move_x =  (image.width - col) * z_block / 11
                if col < image.width:
                    move_x = - move_x

                # transform to base coordinate system
                target.x += current_pose.x + move_x
                target.y += current_pose.y + 20
                target.z += current_pose.z
                target.rx = -90 * math.pi / 180
                target.ry = 0 * math.pi / 180
                target.rz = -90 * math.pi / 180
                break
        return target, current_pose, current_pose.z  - depth[int(row)][int(col)] * 400

    #########################################################################################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm_proxy.closeGripper(...)
    # self.kinovaarm_proxy.getCenterOfTool(...)
    # self.kinovaarm_proxy.openGripper(...)
    # self.kinovaarm_proxy.setCenterOfTool(...)
    # self.kinovaarm_proxy.setGripper(...)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)
