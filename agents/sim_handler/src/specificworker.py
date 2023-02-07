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
from Simulation import *
from scipy.spatial.transform import Rotation as R
import cv2
from scipy import stats
from pynput import keyboard


sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 300

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 194
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)
        self.boxes_ids =  []
        self.already_added = []
        self.updated_cubes = []

        self.suspicious_cubes = []
        self.surprising_event = False

        self.grasped_cube = None
        self.last_grasp   = None
        self.new_grasp = False
        self.grasp_released = False

        self.GRASP_COUNTDOWN_TH = 5
        self.grasp_countdown = 0

        ### Used to reset RT and calibrate from it ###
        # self.update_camera_rt(np.array([ 10, 100, -150, 0, 0, 0]))

        self.sim = Simulation()
        # self.sim.load_scene ("/home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/gen3_cubes.ttt")
        self.sim.load_scene ("/home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/gen3_cubes_2.ttt")
        self.sim.start_simulation()

        # self.sim.insert_hand ("human_hand", [0,0,0], "base")
        self.NUMBER_OF_FINGERS = 2
        self.sim.insert_tips (self.NUMBER_OF_FINGERS, "base")

        self.GRIPPER_ID = self.g.get_node('gripper').id

        self.current_arm_pos = None
        self.dest_arm_pos    = None
        self.update_simulated_arm ()

        self.gripper_target = 1
        self.gripper_state  = 1

        self.object_of_interest = None
        self.last_object_of_interest = None

        self.first_time = True

        self.was_occupied = False
        self.occupied = False

        self.cube_positions = {}

        self.dummie_check = False

        listener = keyboard.Listener(
            on_press=None,
            on_release=self.on_release)
        listener.start()
        self.can_update_sim = True

        self.past_attempts = []
       

        # self.sim.insert_cube ("cube_6", [416, 47,   50], "base")
        # self.sim.insert_cube ("cube_2", [416, 200,   50], "base")

        # self.sim.change_color("Test_cube", (255, 0, 0))

        # self.sim.setJointTargetVelocity(j1,0)

        # self.sim.set_object_pose("goal", [400, 0, 400, np.pi, 0, np.pi/2], "gen3")

        # time.sleep (7)

        # self.sim.set_object_pose("goal", [400, 0, 400, np.pi/2, 0, np.pi/2], "gen3")

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            # signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
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

    def __del__(self):
        """Destructor"""
        self.sim.stop_simulation()
        print ( "Stopped" )

    def setParams(self, params):
        return True

    def on_release(self, key):

        try:
            cube_id = int (key.char)
            cube_id = int (key.char)
            if cube_id == 5:
                print ("--> Received update signal")
                self.can_update_sim = True

            # if key.char == 's':
            #     print ("A step was made")
            #     self.step_finished = True
            # self.object_of_interest  = "cube_" + str(cube_id)
            # print ("got to check", self.object_of_interest)

            # self.sim.change_static ("cube_" + str(cube_id), 0)

        except:
            print ("not an int")
        
            
        
        return True

    def update_camera_rt (self, rt_gr_cam):
        rt = rt_api(self.g)

        griper = self.g.get_node ("gripper")
        h_camera = self.g.get_node ("hand_camera")
        rt.insert_or_assign_edge_RT(griper, h_camera.id, rt_gr_cam[:3], rt_gr_cam[3:])
        self.g.update_node(griper)

    def evaluate_surprising_position (self, cubes):
        print ("suspicious", cubes)

        # "Grasped cube can be anywhere"
        if cubes[0] == self.grasped_cube:
            return

        c4 = self.g.get_node (cubes[0])
        son = None
        for e in c4.edges:
            if e[1] == 'on':
                son = e[0]
        if son is not None:
            lower_cube = self.g.get_node(son)
            print ("It is over", lower_cube.name)
            print ("Testing", c4.name, "on top of", lower_cube.name)


            tf = inner_api(self.g)
            high_pos = tf.transform_axis ("world", c4.name)

            int_rot = high_pos[3:]
            ext_rot = R.from_euler('XYZ', int_rot).as_euler('xyz')
            high_pos[3:] = ext_rot

            low_pos = np.copy(high_pos)
            low_pos [2] -= 20

            print ("New poses are:")
            print (high_pos)
            print (low_pos)

            self.sim.set_multiple_objects_poses ([c4.name, lower_cube.name], [high_pos, low_pos], "base")
            

        else:
            
            print ("It is not over other cube")
            on_edges = self.g.get_edges_by_type ("on")
            for e in on_edges:
                print (e.destination)
                print (e.origin)

                if e.destination == c4.id:
                    print ("But it is should be under. It is not")
                    self.g.delete_edge (self.g.get_node(e.origin).id, e.destination, "on")
                
            # print ("Please, pick", c4.name, "and place it on the table")
            # for i in range (10):
            #     print (10-i, "...")
            #     time.sleep(1)
            # print ("Time is up")

            

    @QtCore.Slot()
    def compute(self):

        # print('SpecificWorker.compute...')


        # goal = self.sim.get_object_pose("goal")
        # rot = R.from_quat(goal[1]).as_euler('xyz')
        # print (np.degrees(rot))

        # return True

        # if not self.can_update_sim:
        #     return True

        if self.surprising_event:
            print ('I should resolve the situation with', self.suspicious_cubes)
            self.evaluate_surprising_position (self.suspicious_cubes)
            self.surprising_event = False


        if self.was_occupied != self.occupied:
            if self.was_occupied:
                print ("------- Stop simulated robot -------")
                self.stop_moving ()
            self.was_occupied  = self.occupied

        # if not np.array_equal(self.current_arm_pos, self.dest_arm_pos):
        #     self.move_to_goal(self.dest_arm_pos)
        #     self.current_arm_pos = np.copy(self.dest_arm_pos)
        
        #### update from arm feedback ####
        # self.update_simulated_arm ()

        if not self.gripper_state == self.gripper_target:
            if self.gripper_target < 0.5:
                self.sim.close_gripper ()
                print ("Close command")

                gr_speed_1 = 1
                gr_speed_2 = 110
                smoothness = 0.75
                while gr_speed_1 > 0.001 and gr_speed_2 > 0.001:
                    r_1, r_2 = self.sim.get_gripper_vel()
                    gr_speed_1 = abs(r_1) * smoothness + gr_speed_1 * (1-smoothness)
                    gr_speed_2 = abs(r_2) * smoothness + gr_speed_2 * (1-smoothness)
                    # print ("grip vel", gr_speed_1, gr_speed_2)
                
                self.sim.stop_gripper ()
            else:
                self.sim.open_gripper ()
                print ("Open command")

            self.gripper_state = self.gripper_target
        
        names = []
        poses = []
        if self.updated_cubes: # and self.can_update_sim:
        # if self.updated_cubes:
            if self.current_arm_pos is not None : #and self.current_arm_pos[2] > 345:
                for id in self.updated_cubes:

                    # if id in self.suspicious_cubes:
                    #     print ("Wont update", id, "something is wrong")
                    #     continue

                    # print ("Updating cube", id)
                    cube = self.g.get_node (id)
                    tf = inner_api(self.g)
                    if cube:
                        pos = tf.transform_axis ("world", cube.name)

                        if pos is None:
                            continue
                        
                        ### Trying to get all rts ######
                        # rt = rt_api(self.g)
                        # edge = self.g.get_edge ("world", cube.name, "RT")
                        # print ("a verte", rt.get_edge_RT_as_rtmat (edge))

                        #### when using Dani's inserter #####
                        # pos[2] -= 20

                        int_rot = pos[3:]
                        ext_rot = R.from_euler('XYZ', int_rot).as_euler('xyz')
                        pos[3:] = ext_rot
                        if id not in self.already_added:
                            
                            
                            #TODO rollback to work with box
                            # if cube.name == "cube_1":
                            #     self.sim.insert_box (cube.name, pos[:3], "base", [0.170, 0.170, 0.150])
                            # elif cube.name == "cube_2":
                            #     self.sim.insert_box (cube.name, pos[:3], "base", [0.145, 0.145, 0.140])
                            # elif cube.name == "cube_5":
                            #     self.sim.insert_box (cube.name, pos[:3], "base", [0.075, 0.095, 0.045])
                            # else:
                            self.sim.insert_cube (cube.name, pos[:3], "base")
                        

                            self.already_added.append(id)
                            print ("Created new cube", id)
                            names.append (cube.name)
                            poses.append (pos)

                            self.cube_positions[id] = pos
                    # else:
                    #     # pass
                        current_pos = self.cube_positions[id]
                        new_pos = pos
                        pos_diff = np.linalg.norm (new_pos[:3]-current_pos[:3])
                        rot_diff = np.linalg.norm (new_pos[3:]-current_pos[3:])
                        if pos_diff > 2 or rot_diff > 0.1 or cube.name == self.grasped_cube:
                            names.append (cube.name)
                            poses.append (pos)
                            self.cube_positions[id] = pos
                            # print ("-->Updating", id, pos_diff, rot_diff)
                            # print ("poses", new_pos, current_pos)

                        # self.sim.set_object_pose(cube.name, pos, "base")


                # print ("Updating simulation")
                if len(names) > 0:
                    print ("---> updating positions of", names)
                    self.sim.set_multiple_objects_poses (names, poses, "base")
                    self.can_update_sim = False


                self.updated_cubes = []
            else:
                print ("Wont trust april tags, arm pos is", self.current_arm_pos)
        
        
        #### grasp detection w/distance ####
        if self.last_grasp != self.grasped_cube:
            if self.grasped_cube is None:
                self.cube_released (self.last_grasp)
            else:
                self.cube_grasped (self.grasped_cube)
            self.last_grasp = self.grasped_cube
        #####################################

        # time.sleep(1)
        print ("---> updating beliefs")
        # now = time.time()
        self.update_cubes_beliefs ()

        # print ("Beliefs", time.time()-now)

        ################################
        ####### Update human hand ######

        # self.update_hand()
        
        ################################

        self.depth = np.frombuffer(self.depth_raw, dtype=np.uint16)
        self.depth = self.depth.reshape((480, 640))
        # self.depth_show = cv2.applyColorMap(cv2.convertScaleAbs(self.depth, alpha=0.03), cv2.COLORMAP_HSV)
        # cv2.imshow("depth", self.depth_show)
        # cv2.waitKey(1)


        # self.check_cube_visibility ()

        if self.object_of_interest != self.last_object_of_interest:
            if self.last_object_of_interest is not None:
                self.sim.change_color(self.last_object_of_interest, (255, 255, 255))
            print ("Should change color of", self.object_of_interest)
            self.sim.change_color(self.object_of_interest, (255, 0, 0))
            self.last_object_of_interest = self.object_of_interest  

        ############# Test if OOI is removable ########################################
        # if self.object_of_interest != self.last_object_of_interest:
        #     if self.last_object_of_interest is not None:

        #         self.sim.change_color(self.last_object_of_interest, (255, 255, 255))
        #         time.sleep(0.05)
        #         last_int_cube = self.g.get_node(self.last_object_of_interest)
        #         near_edges = self.g.get_edges_by_type ("is_near")
        #         for e in near_edges:
        #             self.g.delete_edge (last_int_cube.id, e.destination, "is_near")
            
        #     self.sim.change_color(self.object_of_interest, (255, 0, 0))
            
        #     int_cube = self.g.get_node(self.object_of_interest)
        #     touching = self.sim.get_colliding_objects (self.object_of_interest)
        #     for c in touching:
        #         touching_cube = self.g.get_node(c)
        #         near_e = Edge (touching_cube.id, int_cube.id, "is_near", self.agent_id)
        #         self.g.insert_or_assign_edge (near_e)
        ##################################################################################

        #     print ("removable: ", self.sim.check_if_removable (self.object_of_interest))

        #     self.last_object_of_interest = self.object_of_interest

        # print (" - - - - ")


        return True


    def check_cube_visibility (self):
        
        res, pos, pred_depth = self.sim.check_cube_visibility ()
        if res:
            self.depth_show = cv2.drawMarker(self.depth_show, (pos[0], pos[1]), color=(255, 255, 255), markerType=cv2.MARKER_CROSS, thickness=2)
            self.depth_show = cv2.rectangle(self.depth_show, (pos[0]-20, pos[1]-20),(pos[0]+20, pos[1]+20), color=(255, 255, 255), thickness=2)

            # table_distance_from_tip = np.mean(self.depth) # stats.mode(self.depth, axis=None)
            # roi_mode = np.mean(self.depth[pos[1]-20:pos[1]+20, pos[0]-20:pos[0]+20]) #stats.mode(self.depth[pos[0]-20:pos[0]+20, pos[1]-20:pos[1]+20], axis=None)

            diff = self.depth[pos[1], pos[0]] - pred_depth*1000

            if self.first_time and diff < 50:
                self.first_time = False
                print ("- - - - FOUND RT AT - - - -")
                print ("diff =", diff)
                # pos, rot = self.sim.get_object_pose("tip")

                # rot = R.from_quat(rot).as_euler('xyz')
                # rot = np.multiply(rot, -1)

                # print (pos, rot)

                print ("OOI", self.object_of_interest)

                cube_pos, cube_rot = self.sim.get_object_pose(self.object_of_interest)
                cube_rot = R.from_quat(cube_rot).as_euler('xyz')
                cube_rot = np.multiply(cube_rot, -1)
                # print (cube_pos, cube_rot, np.degrees(cube_rot))

                rt = rt_api(self.g)
                world = self.g.get_node ("world")
                cube_node = self.g.get_node (self.object_of_interest)
                rt.insert_or_assign_edge_RT(world, cube_node.id, cube_pos, cube_rot)
                self.g.update_node(world)


                # gripper = self.g.get_node ("gripper")
                # gripper.attrs["target"].value = [400, 0, 399, 3.14159, 0, -1.57089]
                # self.g.update_node(gripper)
                # print (table_distance_from_tip, roi_mode)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    def update_simulated_arm (self):
        # print ("---> Updating arm")
        # arm_node = self.g.get_node ("arm")
        # joints = arm_node.attrs['robot_local_angular_velocity'].value
        # diffs = [90, 0, 180, 65, 0, 310, 0]

        # for i in range (7):
        #     joints[i] = (joints[i]+diffs[i])%360
        # self.sim.set_joints (joints.tolist())

        
        tf = inner_api(self.g)
        new_pos = tf.transform_axis ("world", "gripper")
        self.set_arm_position (new_pos)

        self.dest_arm_pos = new_pos
        self.current_arm_pos = np.copy(self.dest_arm_pos)

    def set_arm_position (self, pos):
        # print ("before transformation", pos)

        int_rot = pos[3:]
        ext_rot = R.from_euler('XYZ', int_rot).as_euler('xyz')
        pos[3:] = ext_rot

        # print ("set pose", pos)
        self.sim.set_object_pose("goal", pos, "base")
        # self.sim.set_arm_position (new_pos)

    def move_to_goal (self, pos):
        # print ("before transformation mtg", pos)

        int_rot = pos[3:]
        ext_rot = R.from_euler('XYZ', int_rot).as_euler('xyz')
        pos[3:] = ext_rot

        # print ("moving towards new goal")

        # pos[3], pos[4] = pos[4], pos[3]

        self.sim.set_object_pose("goal", pos, "base")

    def stop_moving(self):
        self.sim.stop_moving()


    def update_cubes_beliefs (self):
        # print ("--- beliefs ---")

        # names = [self.g.get_node(id).name for id in self.boxes_ids]

        # pos_rots = self.sim.get_multiple_object_poses (names)

        # for pos, rot in pos_rots:
            

        for id in self.boxes_ids:
            cube = self.g.get_node(id)

            pos, rot = self.sim.get_object_pose(cube.name)

            rot = R.from_quat(rot).as_euler('xyz')

            rot = np.multiply(rot, -1)

            world = self.g.get_node("world")

            v_rt = Edge (cube.id, world.id, "virtual_RT", self.agent_id)
            
            v_rt.attrs["rt_rotation_euler_xyz"] = Attribute(rot, self.agent_id)
            v_rt.attrs["rt_translation"]        = Attribute(pos, self.agent_id)
            self.g.insert_or_assign_edge (v_rt)

    def update_hand (self):
        tf = inner_api(self.g)

        hand = self.g.get_node ("human_hand")

        if hand is None:
            self.sim.update_fingertips ([[0,0,0] for _ in range(self.NUMBER_OF_FINGERS)], "base", False)
            self.grasped_cube = None
            return

        finger_nodes = self.g.get_nodes_by_type ("right_hand")
        
        positions = []
        for f in finger_nodes:
            pos = tf.transform_axis ("world", f.name)
            positions.append(pos)
        
        colliding = self.sim.update_fingertips(positions, "base")
        print ("Colliding", colliding)
        colliding = None if len(colliding) == 0 else colliding[0]

        if colliding:
            cube = self.g.get_node (colliding)
            g_rt = Edge (cube.id, hand.id, "graspping", self.agent_id)
            self.g.insert_or_assign_edge (g_rt)
            self.grasp_countdown = self.GRASP_COUNTDOWN_TH
            self.grasped_cube = colliding
        elif self.last_grasp:
            if self.grasp_countdown > 0:
                print ("Grasp cooldown", self.grasp_countdown)
                self.grasp_countdown -= 1
            else:
                cube = self.g.get_node (self.last_grasp)
                self.g.delete_edge (hand.id, cube.id, "graspping")
        

                self.grasped_cube = colliding

        
        # hand = self.g.get_node ("human_hand")

        # if hand is None:
        #     self.sim.set_object_pose ("human_hand", [0,0,0,0,0,0], "base")
        #     self.grasped_cube = None
        #     return

        # pos = tf.transform_axis ("world", "human_hand")
        # self.sim.set_object_pose ("human_hand", pos, "base")
        
        # colliding = self.sim.check_colisions("human_hand")
        
        # if colliding:
        #     cube = self.g.get_node (colliding)
        #     g_rt = Edge (cube.id, hand.id, "graspping", self.agent_id)
        #     self.g.insert_or_assign_edge (g_rt)
        # elif self.last_grasp:
        #     cube = self.g.get_node (self.last_grasp)
        #     self.g.delete_edge (hand.id, cube.id, "graspping")

        # self.grasped_cube = colliding


    def cube_grasped (self, name):
        print ("Changing to static", name)
        self.sim.change_static (name, 1)

    def cube_released (self, name):
        print ("Changing to dynamic", name)
        self.grasped_cube = None
        self.sim.change_static (name, 0)

    # =============== DSR SLOTS  ================
    # ===========================================

    def update_node_att(self, id: int, attribute_names: [str]):
        if id == self.GRIPPER_ID and 'target' in attribute_names:
            updated_node = self.g.get_node(id)
            target_position  = updated_node.attrs['target'].value
            print ("Received target position", target_position)
            self.dest_arm_pos = target_position
            # self.move_arm_to (target_position)

        if id == self.GRIPPER_ID and 'gripper_target_finger_distance' in attribute_names:
            updated_node = self.g.get_node(id)
            target_distance  = updated_node.attrs['gripper_target_finger_distance'].value
            self.gripper_target = target_distance
            print ("Got new gripper", target_distance)

        if id == self.GRIPPER_ID and 'robot_occupied' in attribute_names: 
            updated_node = self.g.get_node(id)
            self.occupied  = updated_node.attrs['robot_occupied'].value
                

        # if 'active_agent' in attribute_names:
        #     updated_node = self.g.get_node(id)
        #     print ("interest received for", updated_node.name)
        #     self.object_of_interest = updated_node.name
        #     self.first_time = True

        if 'active_agent' in attribute_names:
            dest = self.g.get_node(id)

            ## This causes it to be red
            # self.object_of_interest = dest.name
            # self.first_time = True

            self.can_update_sim = True
            active =  dest.attrs['active_agent'].value
            if (dest.name not in self.suspicious_cubes) and active:
                self.suspicious_cubes.append (dest.name)
                self.surprising_event = True

            if not active and dest.name in self.suspicious_cubes:
                self.suspicious_cubes.remove(dest.name)

    def update_node(self, id: int, type: str):
        if type=='rgbd' and id == 62842907933016084:
            self.has_image = True
            
            updated_node = self.g.get_node(id)
            self.depth_raw = updated_node.attrs['cam_depth'].value

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        pass
        # USED TO UPDATE CUBES -- RESTORE WHEN NOT USING SURPRISE
        dest = self.g.get_node(to)
        if dest.type == 'box' and type == "RT" and dest.name[-1] != '*':
            # print ("Updated edge to", dest.name)
            if (dest.name not in self.updated_cubes):
                self.updated_cubes.append (dest.name)
                if (dest.name not in self.boxes_ids):
                    self.boxes_ids.append (dest.name)

        if dest.type == 'box' and type == "graspping" and dest.name[-1] != '*':
            self.grasped_cube = dest.name
            if dest.name != self.grasped_cube:
                self.new_grasp = True
                self.grasped_cube = dest.name

        # console.print(f"UPDATE EDGE: {fr} to {to}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        # console.print(f"DELETED EDGE FROM {fr} to {to} type {type}", style='green')

        dest = self.g.get_node(to)
        if type == "RT" and dest.name in self.already_added:
            print ("I think this is wrong but wont change color")
            # self.object_of_interest = dest.name
        # dest = self.g.get_node(to)
        if dest.type == 'box' and type == "graspping" and dest.name[-1] != '*':
            self.grasped_cube = None
            # self.grasp_released = True
