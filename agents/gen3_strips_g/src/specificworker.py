#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by Daniel Peix del Río
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

from concurrent.futures import thread
from multiprocessing.dummy import current_process
# from turtle import pos
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from attr import Attribute
from cv2 import imshow, transform
from rich.console import Console
from genericworker import *
import interfaces as ifaces

import pyrealsense2 as rs
import numpy as np
import cv2
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import random

from image_processor import *
from planifier import *
from constants import *
import time
from scipy import stats

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        # CONSTANTS
        self.Period = 100
        self.CUBE_PREFIX = 1000
        self.agent_id = 2803

        # EVENTS HANDLING [basic]
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # EVENT HANDLING [pro]
        self.working = False
        self.do_action = False
        self.do_next_action = False
        
        # IMPORTANT ATTRIBUTES
        self.img_proc = ImageProcessor()
        self.planner = Planifier()
        self.g = DSRGraph(0, "gen3_strips", self.agent_id)
        # self.observation_pose = [534.254, -0.419066, 68.6622, 3.13773, -0.802042, -1.58157]
        # self.working_pose = [544, 0, 400, 3.14159, 0, -1.57089]
        self.hand_tag_detection_count = {}
        self.hand_tags = {}

        # PLANNING
        self.init_state = []
        self.end_state = []
        self.ui.generar.clicked.connect(self.final_state)
        self.begin_plan = False
        self.end = True
        self.plan = []
        self.step = 0
        self.tags = []

        self.GRIPPER_ID = self.g.get_node('gripper').id
        self.robot_moving = False
        self.cubes_update_cycles = 0

        self.interest = None

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)


        time.sleep(0.5)
        # color, self.depthImg = self.img_proc.extract_color_and_depth(self.hand_color_raw, self.hand_depth_raw)
        # self.tags, _ = self.img_proc.compute_april_tags(color, self.depthImg, (self.hand_focal_x, self.hand_focal_y))
        # self.init_state, self.cubes = self.planner.create_initial_state(self.tags)

        cube_nodes = self.g.get_nodes_by_type("box")
        cube_names = []
        for c in cube_nodes:
            cube_names.append(c.name[-1])
        print (cube_names)
        self.init_state, self.cubes = self.planner.create_initial_state_cubes(cube_names)

        for _ in range(3):
            self.move_arm(WORKING_POSE)
            print("TO_HOME")
        self.open_gripper()

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):

        if not self.end:
            if self.begin_plan:
                self.begin_plan = False
                self.planner.save_to_file(self.init_state, self.end_state, self.cubes)
                self.planner.exec_planner()
                time.sleep(0.5)
                
                self.plan = self.planner.load_plan()
                self.move_arm(WORKING_POSE)
                self.do_action = True

            else:
                if self.plan != []:
                    current_step = self.plan[self.step]
                    params = current_step[1] if len(current_step) > 1 else None                                                                                                           

                    if self.do_action:
                        if not self.robot_moving:
                            self.move_arm(WORKING_POSE)
                            self.wait_for_rt (params)
                            if current_step[0] == "pick-up":   
                                print("pick")        
                                self.pick_up(params)
                            elif current_step[0] == "put-down":
                                print("put")        
                                self.put_down(params)
                            elif current_step[0] == "stack": 
                                print("stack")                                                                
                                self.stack(params)                     
                            elif current_step[0] == "unstack":  
                                print("unstack")                
                                self.unstack(params)
                            self.step += 1
                            if self.step >= len(self.plan):
                                self.end = True
                                self.end_state[0] = '  (handempty)'
                                self.init_state = self.end_state
                                self.plan = []
                    print (" - - - waiting for visual info - - - Robot moving:", self.robot_moving, "plan", self.plan)
                    time.sleep(1)   
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # ARM MOVEMENTS
    def move_arm(self, pose):
        gripper = self.g.get_node ("gripper")
        gripper.attrs["target"].value = pose
        self.g.update_node(gripper)

    def open_gripper(self):
        gripper = self.g.get_node ("gripper")
        gripper.attrs["gripper_target_finger_distance"].value = OPEN
        self.g.update_node (gripper)
    
    def close_gripper(self):
        gripper = self.g.get_node ("gripper")
        gripper.attrs["gripper_target_finger_distance"].value = CLOSE
        self.g.update_node (gripper)

    def is_in_working_pose(self):
        current_pose = self.g.get_edge ("arm", "gripper", "RT")
        pos_diff = np.linalg.norm (WORKING_POSE[:3]-current_pose.attrs["rt_translation"].value[:3])
        return pos_diff < 0.15

    # EVENT FUNCTION TRIGGERS TODO: REMOVE ###################################################
    def on_press(self, key):
        pass

    def on_release(self, key):
        # print('Key released: {0}'.format(key))
        try:
            if key.char == 'o':
                self.open_gripper()
                return True
            if key.char == 'w':
                self.move_arm(WORKING_POSE)
                return True
            else:
                # print (key, type(key))
                # print (str(key), "cube_"+str(key.char))
                self.set_interest("cube_"+str(key.char), True)
        except:
            pass
            # print("c: close\n o: open\n w:working pose\n 0:put_down\n N:pick_up(N)")

        if key == keyboard.Key.esc:
            return False
    ##########################################################################################
    
    def final_state(self):
        self.end_state = self.planner.create_final_state(self.ui)
        self.begin_plan = True
        self.end = False
        self.step = 0

    # ACTIONS
    def pick_up(self, cube_id, message=None):
        cube_id = cube_id[0]
        print(f"-->PickUp {cube_id}" if message is None else message)
        self.working = True

        dest_pose = self.g.get_edge ("world", "cube_" + str(cube_id), "RT")
        dest_pose = np.concatenate((dest_pose.attrs["rt_translation"].value, dest_pose.attrs["rt_rotation_euler_xyz"].value))
        
        dest_pose [2] += 21 #TODO sacar si no se usa lo de Guille
        
        
        print ("Grabbin in ", dest_pose)

        dest_pose[3] = 0.0
        dest_pose[4] = np.pi

        print ("--- rot ---")
        print(np.degrees(dest_pose[5]))

        # -45 para minizar giro, + 90 para cuadrar con el gripper
        dest_pose[5] = np.degrees(dest_pose[5]) % 90
        if dest_pose[5] < 45:
            dest_pose[5] += 90

        dest_pose[5] -= 45
        dest_pose[5] = (90 - dest_pose[5]) % 90 + 45
        dest_pose[5] = np.radians(dest_pose[5])

        print(np.degrees(dest_pose[5]))
        print (dest_pose)
        print ("--- end rot ---")

        self.open_gripper()
        self.move_arm(dest_pose)
        # TODO added by guille x3
        self.wait_until_done ()

        self.close_gripper()
        self.wait_until_done ()

        # self.move_arm(WORKING_POSE)
        # # self.set_as_interest ("cube_5", True)
        # self.wait_until_done ()

        self.working = False



    def set_interest (self, name, interest):

        print ("Setting interest in cube", name)

        if self.interest and interest:
            cube_1 = self.g.get_node (self.interest)
            cube_1.attrs['active_agent'].value = False
            self.g.update_node(cube_1)

        cube = self.g.get_node (name)
        cube.attrs['active_agent'].value = interest
        self.g.update_node(cube)

        self.interest = name

    def wait_until_done (self):
        # print ("--- waiting for it to start moving ---")
        # while (not self.robot_moving):
        #     pass
        print ("--- waiting for it to stop moving ---")
        time.sleep(0.5)
        while (self.robot_moving):
            pass
        time.sleep(0.5)
        print (" ---- done ----")

    def wait_for_rt (self, to):
        dest_name = "cube_" + str(to[-1])
        print ("waiting for", dest_name)
        self.set_interest (dest_name, True)
        time.sleep(0.5)
        transformation = None
        while (transformation is None):
            transformation = self.g.get_edge ("world", dest_name, "RT")
        # print (transformation)
        print ("got it")
        gripper = self.g.get_node ("gripper")
        gripper.attrs["robot_occupied"].value = False
        self.g.update_node(gripper)
        time.sleep(3)

        # print (transformation)

    def put_down(self, cube_id, message=None, cube_to_stack_in=-1):
        print(f"-->PutDown {cube_id}" if message is None else message)
        self.working = True

        dest_pose = self.__findPose(cube_to_stack_in)
        self.move_arm(dest_pose)
        self.wait_until_done ()

        self.open_gripper()
        self.wait_until_done ()

        # self.move_arm(WORKING_POSE)
        # self.wait_until_done ()

        self.working = False
        pass

    def unstack(self, params):
        cube_id, cube_aux = [params[0]], params[1]
        self.pick_up(cube_id, f"-->Unstack {cube_id} from {cube_aux}")
    
    def stack(self, params):
        cube_id, cube_aux = params[0], params[1]
        self.put_down(cube_id, f"-->Stack {cube_id} on {cube_aux}", cube_aux)

    def __findPose(self, cube_id):
        pos = []
        rot = []

        if cube_id == -1:
            pos, rot = self.__find_free_spot()
            cv2.imshow("CACACACA", self.color)
            cv2.waitKey(5)
        else:
            dest_pose = self.g.get_edge ("world", "cube_" + str(cube_id), "RT")
            pos = dest_pose.attrs["rt_translation"].value.tolist()
            rot = dest_pose.attrs["rt_rotation_euler_xyz"].value.tolist()
            
        pos [2] += 21 #TODO sacar si no se usa lo de Guille
        pos [2] += 50

        rot[0] = 0.0
        rot[1] = np.pi

        print ("--- rot ---")
        print (np.degrees(rot))
        # -45 para minizar giro, + 90 para cuadrar con el gripper
        rot[2] = np.degrees(rot[2]) % 90
        if rot[2] < 45:
            rot[2] += 90

        rot[2] -= 45
        rot[2] = (90 - rot[2]) % 90 + 45
        rot[2] = np.radians(rot[2])

        print(np.degrees(rot))
        print ("--- end rot ---")

        print("AL BRAZO", pos+rot)
        return pos + rot

    def __find_free_spot(self):
        # compute side of box in pixels from current location
        current_pose = self.g.get_edge ("arm", "gripper", "RT")
        current_pose = current_pose.attrs["rt_translation"].value.tolist() + current_pose.attrs["rt_rotation_euler_xyz"].value.tolist()
        current_pose[2] += 150
        side = int(self.hand_focal_x * 90 / current_pose[2]) * 2

        # choose a random rectangle on the table (check with distance from the camera)
        result = False
        counter = 0
        hits = 0
        target = [sys.float_info.max, 0, 0, -90 * np.pi / 180, 0 * np.pi / 180, -90 * np.pi / 180]

        while not result and counter < 30000:
            # image is treated as np array, with rows first

            height = self.color.shape[0]
            width  = self.color.shape[1]

            k = 1
            row = random.randint(side * k, height - side * k) // 2
            col = random.randint(side * k,  width - side * k) // 2
            roi_depth = self.depthImg[row - side // 2 : row + side // 2, col - side // 2 : col + side // 2]
            
            table_distance_from_tip = stats.mode(self.depthImg, axis=None)[0] # TODO Guille changed from roi to whole img

            # check that all points belong to the table
            hits = np.count_nonzero(abs(roi_depth - table_distance_from_tip) < 15)
            counter += 1
            result = hits >= (roi_depth.size * 0.95)
            # print("ROI SHAPE", roi_depth.shape, "ROW", row, "COL", col, "MODA", table_distance_from_tip)
            # print("HITS", hits, "RESULT", result, "COUNTER", counter)

            # compute target coordinates. x, y, z in camera CS: x+ right, y+ up, z+ outwards
            target[0] = - int(( col - width  // 4) * current_pose[2] / self.hand_focal_x)
            target[1] = - int(( row - height // 4) * current_pose[2] / self.hand_focal_y)

            # # transform to tip coordinate system: x+ right, y+ backwards, z+ up
            # target[1] = - target[1] - (-107)
            # target[2] = - current_pose[2]

            # # transform to base coordinate system
            # target[0] += current_pose[0]
            # target[1] += current_pose[1]
            # target[2] += current_pose[2]



        tf = inner_api(self.g)
        print("ANTES", target)
        target = tf.transform_axis ("world", target, "hand_camera").tolist()
        target[2] = 0
        print("DESPUES", target)

        if counter == 30000:
            print("MAX ITER REACHED")
        
        print ("Suposed free spot")
        print(target)
        print("ROW", row, "COL", col, "Color shape", self.color.shape, "depth shape", self.depthImg.shape, "roi shape", roi_depth.shape)
        self.color = cv2.rectangle(self.color, (col*2 - side // 2, row*2 - side // 2),
                                  (col*2 + side // 2, row*2 + side // 2), (255, 0, 0), 6)
        return target[:3], target[3:]
        # return WORKING_POSE[:3], WORKING_POSE[3:]

    # INTERFACE CUBES-DSR
    def cubes_to_dsr(self, tags):
        cubes = []
        for id in tags.keys():
            if id not in self.hand_tag_detection_count.keys():
                self.hand_tag_detection_count[id] = 0

        for id in self.hand_tag_detection_count.keys():
            if id in tags.keys():
                self.hand_tag_detection_count[id] = np.minimum (30, self.hand_tag_detection_count[id]+1)
            else:
                self.hand_tag_detection_count[id] = np.maximum (0, self.hand_tag_detection_count[id]-1)
            
            if (self.hand_tag_detection_count[id] > 20):
                self.hand_tags[id] = tags[id]
                self.__insert_or_update_cube (tags, id)
                cubes.append(id)
            else:
                self.__delete_cube (id)
        return cubes
        

    def __insert_or_update_cube (self, tags, cube_id, offset = 0):
        cube = tags[cube_id]
        cube_name = "cube_" + str(cube_id)
        tf = inner_api(self.g)
        new_pos = tf.transform_axis ("world", cube["pos"] + cube["rot"], "hand_camera")
        
        if (cube_node := self.g.get_node(cube_name)) is None:
            new_node = Node(cube_id + self.CUBE_PREFIX + offset, "box", cube_name)
            new_node.attrs['pos_x'] = Attribute(float(REFERENCE_COORD + CUBE_OFFSET * cube_id), self.agent_id)
            new_node.attrs['pos_y'] = Attribute(float(CUBE_OFFSET), self.agent_id)
            new_node.attrs['active_agent'] = Attribute(False, self.agent_id)
            
            self.g.insert_node (new_node)
            cube_node = new_node

        rt = rt_api(self.g)
        try:
            current_pos = tf.transform_axis ("world", cube_name)
            pos_diff = np.linalg.norm (new_pos[:3]-current_pos[:3])
            rot_diff = np.linalg.norm (new_pos[3:]-current_pos[3:])
            if pos_diff < MAX_POS_DIFF and rot_diff < MAX_ROT_DIFF:
                return
        except:
            print ("Does not exist")

        world = self.g.get_node ("world")
        rt.insert_or_assign_edge_RT(world, cube_node.id, new_pos[:3], new_pos[3:])
        self.g.update_node(world)

    def __delete_cube (self, cube_id):

        if (cube := self.g.get_node ("cube_" + str(cube_id))):
            world = self.g.get_node ("world")
            self.g.delete_edge (world.id, cube.id, "RT")

        ###############   original  ######################
        # if (cube := self.g.get_node ("cube_" + str(cube_id))):
        #     self.g.delete_node (cube.id)

    # DSR SLOTS
    def update_node_att(self, id: int, attribute_names: [str]):
        if id == self.GRIPPER_ID and 'robot_occupied' in attribute_names:
            updated_node = self.g.get_node(id)
            occupied  = updated_node.attrs['robot_occupied'].value
            self.robot_moving = occupied

    def update_node(self, id: int, type: str):
        if type=='rgbd' and id == CAMERA_ID:
            updated_node = self.g.get_node(id)
            self.hand_depth_raw = updated_node.attrs['cam_depth'].value
            self.hand_color_raw = updated_node.attrs['cam_rgb'].value

            self.hand_focal_x = updated_node.attrs['cam_rgb_focalx'].value
            self.hand_focal_y = updated_node.attrs['cam_rgb_focaly'].value

    def delete_node(self, id: int):
        pass

    def update_edge(self, fr: int, to: int, type: str):
        pass

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        pass

    def delete_edge(self, fr: int, to: int, type: str):
        pass
