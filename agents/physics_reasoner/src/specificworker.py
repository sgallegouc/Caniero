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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
from pynput import keyboard


import time
import numpy as np
np.set_printoptions(linewidth=np.inf)
import cv2
import json

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
        self.Period = 50

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 136
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)
        self.hadcoded_poses = [ [330,  -20,  500,     0,  np.pi,  np.pi/2], 
                                [190,  -20,  460,     0,    2.8,  np.pi/2],
                                [320,  156,  460,  2.90,      0, -np.pi/2],
                                [320, -156,  460,  0.35, -np.pi,  np.pi/2]]

        
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

        self.calibration_info ="{"
        self.calibration_trial = 0
        self.can_execute = False

        self.rts = []

        # f = open('/home/robolab-kinova/guille_img/Pruebas_self_cal/cal_rts.txt')
        # data = json.load(f)
        
        
        # for i in data:
        #     self.rts.append(np.array(data[i]))
        self.current_test = -1
        
        self.cube_rts = {}

        # self.b_rt = np.array([22.9, 72.57, -145.05, -0.053, -0.006, 0.00])
        # self.original_rt = np.array([ 17, 107, -150, 0, 0, 0])
        self.original_rt = np.array([ 17, 107, -150, 0, 0, 0])
        self.b_rt = np.copy(self.original_rt)

        self.multiple_trials = []
        self.error_evolution = []
        self.resulting_rts = []


        self.canvas = np.zeros((300, 800))
        self.current_value = 150
        self.avg_index = 0
        self.last_N_avg = [0 for _ in range(20)]

        
        ################################################################
        # self.load_cube_rts()

        # print ("Starting initial optimization")
        # print (self.b_rt, self.transformation_error_2(self.b_rt))

        # ini = time.time()

        # self.b_rt = self.compute_and_publish_best_rt(self.b_rt)
        # print ("Finished initial optimization in", time.time()-ini)
        # self.old_error = self.transformation_error_2(self.b_rt)
        # print (self.b_rt, self.old_error)
        ###############################################################

        # print (self.error_evolution)


        # self.update_camera_rt(np.array([10, 100, -150, 0, 0, 0]))

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

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def on_press(self, key):
        pass
        # try:
        #     print('Alphanumeric key pressed: {0} '.format(
        #         key.char))
        # except AttributeError:
        #     print('special key pressed: {0}'.format(
        #         key))

    def on_release(self, key):
        # print('Key released: {0}'.format(
        #     key))
        
        try:
            cube_id = int (key.char)
            if cube_id == 9:
                self.calibration_info += "}"
                file = open("/home/robolab-kinova/guille_img/Pruebas_self_cal/pruebas_pilar/cal_rts.json", "w+")
                # Saving the array in a text file
                content = str(self.calibration_info)
                file.write(content)
                file.close()
            elif cube_id == 8:
                # print ("--- Saving state")
                # cubes = self.g.get_nodes_by_type ("box")
                # self.calibration_info += "\t\"" + str(len(cubes)) + str(self.calibration_trial%3) + "\" : " + str(self.b_rt.tolist()) + ", \n"
                # self.calibration_trial += 1
                print ("-----> Next rt")
                self.current_test += 1
                self.b_rt = np.array([22.9, 72.57, -145.05, -0.053, -0.006, 0.00]) # self.rts[self.current_test]
                self.update_camera_rt (self.b_rt)
            elif cube_id == 7:
                print ("--- New Pose----")
                self.current_value = 200
                
                # print ("\"" + str(self.current_test) + "-" + str(self.current_pose_id) +"\": {\n \"rt\":", self.b_rt.tolist(), ", \n \"report\":{")
                self.can_execute = True
            else:
                dest_pose = self.hadcoded_poses [cube_id]
                self.current_pose_id = cube_id
                print ("should move to ", dest_pose)
                gripper = self.g.get_node ("gripper")
                gripper.attrs["gripper_target_finger_distance"].value = 1.0
                gripper.attrs["target"].value = dest_pose
                self.g.update_node (gripper)
        except:
            print ("not an int")
            return



    def transformation_error (self, rt_gr_cam):
        rot_m = R.from_euler("XYZ", rt_gr_cam[3:])

        res_mat = np.hstack([rot_m.as_matrix(), rt_gr_cam[:3].reshape(3,1)])
        res_mat = np.vstack([res_mat, [0,0,0,1]])
        tf = inner_api(self.g)
        rmat_w_gr = tf.get_transformation_matrix("world", "gripper")
        mat = rmat_w_gr @ res_mat

        mat2 = np.linalg.inv(mat[:3,:3])


        print (mat,"\n\n", mat2, "\n\n", mat[:3,:3]@mat2, "\n\n", mat2@mat2)

        cubes = self.g.get_nodes_by_type ("box")
        diff = 0
        for cube in cubes:
            cube1 = self.g.get_edge ("hand_camera", cube.name, "RT")
            cube1_trans = cube1.attrs["rt_translation"].value
            cube1_rot   = cube1.attrs["rt_rotation_euler_xyz"].value

            v_rt = self.g.get_edge ("world", cube.name, "virtual_RT")

            print(cube.name, cube1_rot, cube1_rot @ mat2, v_rt.attrs["rt_rotation_euler_xyz"].value)

            cube1_trans = np.append(cube1_trans, 1)
            cube1_res = mat @ cube1_trans


            diff += np.linalg.norm(cube1_res[:3] - v_rt.attrs["rt_translation"].value)
        # print (diff)
        return diff

    def angle_diff (self, v1, v2):

        v1_q = R.from_euler("xyz", v1).as_quat()
        v2_q = R.from_euler("xyz", v2).as_quat()

        return 1 - np.inner(v1_q, v2_q)**2 # np.linalg.norm(v1_q - v2_q)

    def get_cube_error (self, name, verbose=False):
        rt = rt_api(self.g)
        tf = inner_api(self.g)

        rt = tf.transform_axis ("world", self.cube_rts[name], "hand_camera")
        v_rt = self.g.get_edge ("world", name, "virtual_RT")


        rot_diff   = self.angle_diff(rt[3:],  v_rt.attrs["rt_rotation_euler_xyz"].value) * 1000
        trans_diff = np.linalg.norm (rt[:3] - v_rt.attrs["rt_translation"].value)

        # if rot_diff > 200:
        #     print ('For', name)
        #     print ('Ang diff', rt[3:], v_rt.attrs["rt_rotation_euler_xyz"].value)
        #     print ('is', rot_diff)
        #     print ('\n\n')


        return rot_diff, trans_diff


    def update_camera_rt (self, rt_gr_cam):
        rt = rt_api(self.g)

        griper = self.g.get_node ("gripper")
        h_camera = self.g.get_node ("hand_camera")
        rt.insert_or_assign_edge_RT(griper, h_camera.id, rt_gr_cam[:3], rt_gr_cam[3:])
        self.g.update_node(griper)

    def load_cube_rts (self):
        # print  ("--> Updated rts to avoid noise")
        cubes = self.g.get_nodes_by_type ("box")
        tf = inner_api(self.g)
        
        for cube in cubes:
            rt = self.g.get_edge ("hand_camera", cube.name, "RT")
            self.cube_rts[cube.name] = np.concatenate((rt.attrs["rt_translation"].value, rt.attrs["rt_rotation_euler_xyz"].value))

        # print (self.cube_rts)

    def transformation_error_2 (self):

        cubes = self.g.get_nodes_by_type ("box")
        diff = 0
        error_array = []
        for cube in cubes:

            rt = self.g.get_edge ("hand_camera", cube.name, "RT")
            if rt is None:
                cube.attrs['active_agent'].value = False
                self.g.update_node (cube)
                continue
            self.cube_rts[cube.name] = np.concatenate((rt.attrs["rt_translation"].value, rt.attrs["rt_rotation_euler_xyz"].value))

            rot_diff ,trans_diff = self.get_cube_error(cube.name)
            # rt = tf.transform_axis ("world", cube.name)
            # v_rt = self.g.get_edge ("world", cube.name, "virtual_RT")

            # rot_diff   = self.angle_diff(rt[3:],  v_rt.attrs["rt_rotation_euler_xyz"].value) * 1000
            # trans_diff = np.linalg.norm (rt[:3] - v_rt.attrs["rt_translation"].value)


            # print (cube.name, trans_diff, rot_diff)
            e = rot_diff + trans_diff
            error_array.append(e)

            # if e > 150:
            if trans_diff > 10:
                print (cube.name, "is in a surprising pose. E =", rot_diff, trans_diff)
                cube.attrs['active_agent'].value = True
                self.g.update_node (cube)
            else:
                cube.attrs['active_agent'].value = False
                self.g.update_node (cube)
            # elif e > 60:
            #     print ("Yeah, it might be, update", cube.name)

        # self.error_evolution.append(avg)
        return error_array

    def move_to_hardcoded_pose (self, index):
        dest_pose = self.hadcoded_poses [index]

        gripper = self.g.get_node ("gripper")
        gripper.attrs["gripper_target_finger_distance"].value = 1.0
        gripper.attrs["target"].value = dest_pose
        self.g.update_node (gripper)

    def compute_and_publish_best_rt (self, initial_guess):
        rt = rt_api(self.g)

        res = minimize(self.transformation_error_2, initial_guess, method='Powell', tol=1e-5, options={"maxfev":1000})
        
        # griper = self.g.get_node ("gripper")
        # h_camera = self.g.get_node ("hand_camera")
        # rt.insert_or_assign_edge_RT(griper, h_camera.id, res.x[:3], res.x[3:])
        # self.g.update_node(griper)
        print ("optimization success:", res)
        self.resulting_rts.append(res.x)
        return res.x

    @QtCore.Slot()
    def compute(self):
        errors = self.transformation_error_2()
        self.current_value = int (np.mean(errors))
        self.current_value = np.clip (self.current_value, 0, 299) 

        column = np.zeros((300,1))
        column[self.current_value] = 1

        self.last_N_avg[self.avg_index] = self.current_value
        self.avg_index = (self.avg_index + 1) % len(self.last_N_avg)


        # self.current_value += np.random.randint(-1, 2)

        self.canvas = np.delete(self.canvas, 0, 1) 
        self.canvas = np.append(self.canvas, column, axis=1)

        img = cv2.flip(self.canvas, 0)

        img = cv2.putText(img, str(np.mean(self.last_N_avg)), (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, 1, 1, cv2.LINE_AA)

        cv2.imshow("canvas", img)

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    def plot_bars (self, img, values, max, steps, signed = 1):
        height = img.shape[0]
        width  = img.shape[1]
        cant = len(values)
        size = width//cant

        for i in range(max//steps):
            h = ((i*steps) * height) // max
            cv2.line(img, (0, height-h), (width, height-h), (255,255,255), 1)

        offset = 0
        for i in range(len(values)):
            x1 = offset + size//3
            x2 = offset + (2*size)//3 
            y = height - ((int (values[i]) * height) // max)

            # cv2.putText(img, (x1, 100), names[i], cv2.FONT_HERSHEY_SIMPLEX, 1, (150, 0, 0), 1, 2)
            cv2.rectangle(img, (x1, y),(x2, height//signed), (255, 0, 255), -1)
            offset += size

    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
