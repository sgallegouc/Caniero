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
from planifier import *
import time
from pynput import keyboard


sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


POSE_FILE = '/home/guille/catkin_ws/src/arm_controller_g/dest_pose.arm'
GRIPPER_FILE = '/home/guille/catkin_ws/src/arm_controller_g/gripper_pose.arm'

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 500

        self.agent_id = 183
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)


        self.planner = Planifier()
        self.init_state = []
        self.end_state = []
        
        self.step_finished = False
        self.cancel_plan = True
        self.delete_as_planned = False

        listener = keyboard.Listener(
            on_press=None,
            on_release=self.on_release)
        listener.start()

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            # signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            # signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            # signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            # signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)


        # time.sleep(0.5)

        # ########################## Example plan generation ##################################
        # cube_nodes = self.g.get_nodes_by_type("box")
        # cube_names = []
        # for c in cube_nodes:
        #     cube_names.append(c.name[-1])
        # print (cube_names)
        # self.init_state, self.cubes = self.planner.create_initial_state_cubes(cube_names)
        # self.end_state = self.planner.create_final_state([])
        # time.sleep(0.5)

        # self.planner.save_to_file(self.init_state, self.end_state, self.cubes)
        # self.planner.exec_planner()
        # time.sleep(0.5)
        
        # self.plan = self.planner.load_plan()
        # print (self.plan)
        # ####################################################################################

        

    def __del__(self):
        """Destructor"""

    def on_release(self, key):

        
        # if key.char == 'l':
        #     c4 = self.g.get_node ("cube_4")
        #     for e in c4.edges:
        #         if e[1] == 'on':
        #             son = e[0]
        #     lower_cube = self.g.get_node(son)
        
        try:

            if key.char == 'j':
                upper = self.g.get_node ("cube_4")
                lower = self.g.get_node ("cube_3")
                on_e = Edge (lower.id, upper.id, "on", self.agent_id)
                self.g.insert_or_assign_edge (on_e)

            if key.char == 'c':
                self.cancel_plan = True
            
            if key.char == 's':
                print ("A step was made")
                self.step_finished = True

        except:
            print ("Not a valid character")

    

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def impact_effects (self, step):
        action = step[0]
        params = step[1]

        if action == 'pick-up':
            self.pick_cube ("cube_" + str(params[0]))
            

        if action == 'stack':
            self.stack_on ("cube_" + str(params[1]))
            upper = self.g.get_node ("cube_" + str(params[0]))
            lower = self.g.get_node ("cube_" + str(params[1]))

            on_e = Edge (lower.id, upper.id, "on", self.agent_id)
            
            hand = self.g.get_node ("gripper")
            self.g.delete_edge (hand.id, upper.id, "graspping")
            self.g.insert_or_assign_edge (on_e)

        if action == 'unstack':
            self.delete_as_planned = True
            upper = self.g.get_node ("cube_" + str(params[0]))
            lower = self.g.get_node ("cube_" + str(params[1]))
            self.g.delete_edge (upper.id, lower.id, "on")

    def pick_cube (self, name):
        print ("Will pick up", name)

        self.go_to_cube(name)
        time.sleep(5)
        self.move_gripper(0.9)
        time.sleep(5)
        cube = self.g.get_node (name)
        hand = self.g.get_node ("gripper")
        g_rt = Edge (cube.id, hand.id, "graspping", self.agent_id)
        self.g.insert_or_assign_edge (g_rt)
        self.go_above_cube(name, 100)
        time.sleep(5)
        self.go_to_home()
        time.sleep(5)

    def stack_on (self, name):
        print ("Will stack on", name)
        self.go_above_cube(name, 50)
        time.sleep(5)
        self.move_gripper(0.0)
        time.sleep(5)
        self.go_above_cube(name, 100)
        time.sleep(5)
        self.go_to_home()
        time.sleep(5)


    def go_to_cube (self, name):

        v_rt = self.g.get_edge ("world", name, "virtual_RT")

        # rot_diff   = self.angle_diff(rt[3:],  v_rt.attrs["rt_rotation_euler_xyz"].value) * 1000
        # trans_diff = np.linalg.norm (rt[:3] - v_rt.attrs["rt_translation"].value)

        # print (v_rt.attrs["rt_translation"].value, v_rt.attrs["rt_rotation_euler_xyz"].value)
        dest_v_rt = list(v_rt.attrs["rt_translation"].value) + list(v_rt.attrs["rt_rotation_euler_xyz"].value)

        tf = inner_api(self.g)
        cube_pos = tf.transform_axis ("arm", dest_v_rt, "world")

        # cube_pos[2] = 20
        trans_p = np.array(cube_pos[:3]) / 1000
        
        print (cube_pos)

        f = open(POSE_FILE, "w")
        formated_pose = ''
        for i in trans_p:
            formated_pose += str(i) + ', '
        
        formated_pose += '0, 20, 0'

        f.write(formated_pose)
        f.close()

    def go_above_cube (self, name, amount):

        v_rt = self.g.get_edge ("world", name, "virtual_RT")

        # rot_diff   = self.angle_diff(rt[3:],  v_rt.attrs["rt_rotation_euler_xyz"].value) * 1000
        # trans_diff = np.linalg.norm (rt[:3] - v_rt.attrs["rt_translation"].value)

        # print (v_rt.attrs["rt_translation"].value, v_rt.attrs["rt_rotation_euler_xyz"].value)
        dest_v_rt = list(v_rt.attrs["rt_translation"].value) + list(v_rt.attrs["rt_rotation_euler_xyz"].value)

        tf = inner_api(self.g)
        cube_pos = tf.transform_axis ("arm", dest_v_rt, "world")

        cube_pos[2] += amount
        trans_p = np.array(cube_pos[:3]) / 1000
        
        print (cube_pos)

        f = open(POSE_FILE, "w")
        formated_pose = ''
        for i in trans_p:
            formated_pose += str(i) + ', '
        
        formated_pose += '0, 20, 0'

        f.write(formated_pose)
        f.close()

    def move_gripper (self, pos):
        f = open(GRIPPER_FILE, "w")
        f.write(str(pos))
        f.close()

    def go_to_home (self):

        print ("going home")

        f = open(POSE_FILE, "w")
        
        formated_pose = '0.4, 0, 0.1, 0, 20, 0'

        f.write(formated_pose)
        f.close()

    def get_current_state (self):

        cube_nodes = self.g.get_nodes_by_type("box")
        cubes = []
        clear = []        

        for c in cube_nodes:
            son = None
            for e in c.edges:
                if e[1] == 'on':
                    son = e[0]
            if son is not None:
                cubes.append(["on", c.name[-1], self.g.get_node(son).name[-1]])
            clear.append(c.name[-1])

        belowers = np.array(cubes)[:,2] if cubes else []
        uppers   = np.array(cubes)[:,1] if cubes else []

        for c in clear:
            if c not in belowers:
                cubes.append(["clear", c])
            if c not in uppers:
                cubes.append(["table", c])


        return cubes

    @QtCore.Slot()
    def compute(self):

        if self.cancel_plan:
            self.cancel_plan = False
            print ("Gonna make a plan...")

            st = self.get_current_state()
            print(st)
            print ('-----------------')

            self.init_state, self.cubes = self.planner.create_current_state(st)
            self.end_state = self.planner.create_final_state([])
            print(self.cubes)
            time.sleep(0.1)

            self.planner.save_to_file(self.init_state, self.end_state, self.cubes)
            self.planner.exec_planner()
            time.sleep(0.1)
            
            self.plan = self.planner.load_plan()

            print("Ok, there we go \n", self.plan)
            print('My plan is to ')
            print (self.plan)

        if self.step_finished:

            print ("Well done")
            self.step_finished = False
            self.impact_effects(self.plan[0])
            self.plan = self.plan[1:]
            if self.plan:
                print ("now, execute", self.plan[0])
            else:
                print ("Done with the plan!")


        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)






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
        dest = self.g.get_node(to)
        if type == "on":
            print ("Deleted edge, cancel plan")
            if self.delete_as_planned:
                self.delete_as_planned = False
            else:
                self.cancel_plan = True
