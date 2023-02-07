from os import stat
import sys
import matplotlib.pyplot as plt
import numpy as np
sys.path.append('/home/robocomp/software/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/zmqRemoteApi/clients/python')
from zmqRemoteApi import RemoteAPIClient
import time
from scipy.spatial.transform import Rotation as R



class Simulation():

    def __init__(self):

        client = RemoteAPIClient()
        self.sim = client.getObject('sim')
        self.handles = {}
        # TODO: Relative path
        # sim.loadScene("/home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/gen3_cubes.ttt")
        self.SCRIPTED_OBJ = "base_link_visual0"
        print('Scene loaded')
        
        # block = sim.getObjectHandle ('bloque_3')
        # pos = sim.getObjectPosition(block, rs_zero)


        # for i in range (30):
        #     time.sleep(1)
        #     pos[2] -= 0.01
        #     pos [0] += 0.05
        #     sim.setObjectPosition (block, rs_zero, pos)

    def stop_simulation (self):
        self.sim.stopSimulation()

    def __del__(self):
        print ("stopping")
        self.stop_simulation()

    def load_scene (self, path):
        self.sim.loadScene (path)
        self.handles["goal"] = self.sim.getObjectHandle("goal")

    def start_simulation (self):
        self.sim.startSimulation()

    def insert_hand (self, name, pose, parent):
        
        # DSR (mm) to Coppelia (m)
        position = np.multiply(pose[:3], 0.001).tolist()

        size = 0.05

        relative_to = self.sim.getObjectHandle(parent) #'gen3'
        hand = self.sim.createPureShape (1, 24, [size, size, size], 1)
        self.sim.setObjectPosition(hand, relative_to, position)

        self.handles[name] = hand

    def insert_tips (self, cant, parent):
        
        # DSR (mm) to Coppelia (m)
        position = [0,0,0]
        size = 0.02

        relative_to = self.sim.getObjectHandle(parent) #'gen3'
        for i in range(cant):
            name = "finger_" + str(i)
            tip = self.sim.createPureShape (1, 16, [size, size, size], 1)
            self.sim.setObjectPosition(tip, relative_to, position)

            self.sim.setShapeColor(tip, None, self.sim.colorcomponent_ambient_diffuse, [0.6, 0.44, 0.4])
            self.sim.setShapeColor(tip, None, self.sim.colorcomponent_transparency, [0.3])
            print ("inserted", name)

            self.handles[name] = tip

    def insert_cube (self, name, pose, parent):
        
        # DSR (mm) to Coppelia (m)
        position = np.multiply(pose[:3], 0.001).tolist()

        relative_to = self.sim.getObjectHandle(parent) #'gen3'
        block2 = self.sim.createPureShape (0, 8, [0.04, 0.04, 0.04], 0.1)
        # block2 = self.sim.createPureShape (0, 24, [0.04, 0.04, 0.04], 0.1)
        
        self.sim.setObjectSpecialProperty(block2, self.sim.objectspecialproperty_collidable)
        self.sim.setObjectPosition(block2, relative_to, position)

        self.handles[name] = block2

        print ("Inserted", name, "as", block2)


    def remove_object (self, name):
        obj = self.handles[name]
        self.sim.removeObject (obj)
        del self.handles[name]

    
    def get_colliding_objects (self, name):
        obj = self.handles[name]
        names = []
        touching = self.sim.callScriptFunction ("get_touching_objects@gen3", 1, obj).keys()

        for n in self.handles:
            if self.handles[n] in touching:
                names.append(n)

        return names

    def check_if_removable (self, name):
        obj = self.handles[name]
        self.sim.callScriptFunction ("test_stability_without_1@gen3", 1, obj)
        time.sleep(1)
        diff  = self.sim.callScriptFunction ("test_stability_without_2@gen3", 1, obj)

        return diff < 0.001

    def check_colisions (self, name1):

        ob1 = self.handles[name1]
        # ob2 = self.handles[name2]
        # try:
        colliding = None
        for name in self.handles:
            if "cube" not in name:
                continue
            ob2 = self.handles[name]
            res = self.sim.checkCollision (ob1, ob2) # self.sim.handle_all)
            if res and res[0] == 1:
                colliding = name

        return colliding 
        # except:
        #     print ("Couldnt get collisions")
        #     return 0,  []

    
    def update_fingertips (self, positions, parent, detected = True):
        names = []
        finger_handles = [self.handles["finger_" + str(i)] for i in range(len(positions))]

        if detected:   
            sim_positions = [np.multiply(p[:3], 0.001).tolist() for p in positions]
            grasping = self.sim.callScriptFunction ("update_fingertips@gen3", 1, sim_positions, finger_handles, parent)

            for n in self.handles:
                if self.handles[n] in grasping:
                    names.append(n)
        else:
            self.sim.callScriptFunction ("update_fingertips@gen3", 1, positions, finger_handles, parent)

        return names



    def change_static (self, name, is_static):

        block = self.handles[name]
        self.sim.setObjectInt32Param(block, self.sim.shapeintparam_static, is_static)
        

    def insert_box (self, name, pose, parent, dims=[0.18, 0.08, 0.12]):

        # dims = [0.18, 0.08, 0.12]
        wall_th = 0.001
        
        # DSR (mm) to Coppelia (m)
        position = np.multiply(pose[:3], 0.001).tolist()
        relative_to = self.sim.getObjectHandle(parent) #'gen3'

        base   = self.sim.createPureShape (0, 16, [dims[0], dims[1], wall_th], 0.1)
        left   = self.sim.createPureShape (0, 16, [wall_th, dims[1], dims[2]], 0.1)
        right  = self.sim.createPureShape (0, 16, [wall_th, dims[1], dims[2]], 0.1)
        back   = self.sim.createPureShape (0, 16, [dims[0], wall_th, dims[2]], 0.1)
        front  = self.sim.createPureShape (0, 16, [dims[0], wall_th, dims[2]], 0.1)

        base_position = [position[0], position[1], position[2]+(dims[2]/2)]
        self.sim.setObjectPosition(base, relative_to, base_position)

        back_position = [position[0]-(dims[1]/2), position[1], position[2]]
        self.sim.setObjectPosition(back, relative_to, back_position)

        front_position = [position[0]+(dims[1]/2), position[1], position[2]]
        self.sim.setObjectPosition(front, relative_to, front_position)

        left_position = [position[0], position[1]-(dims[0]/2), position[2]]
        self.sim.setObjectPosition(left, relative_to, left_position)

        right_position = [position[0], position[1]+(dims[0]/2), position[2]]
        self.sim.setObjectPosition(right, relative_to, right_position)

        box = self.sim.groupShapes ([base, back, front, left, right])
        self.sim.setObjectInt32Param(box,self.sim.shapeintparam_static, 0)
        self.sim.setObjectInt32Param(box,self.sim.shapeintparam_respondable, 1)
        self.sim.setObjectSpecialProperty(box, self.sim.objectspecialproperty_collidable | self.sim.objectspecialproperty_detectable | self.sim.objectspecialproperty_measurable)
        # self.sim.setShapeColor (box, None, self.sim.colorcomponent_ambient_diffuse, (255, 0, 0))

        self.sim.setObjectInt32Param(box, self.sim.objintparam_visibility_layer, 257) 


        or_ref = self.sim.getObjectHandle("box_orientation")

        self.sim.reorientShapeBoundingBox(box, or_ref)

        self.sim.resetDynamicObject(box)

        self.handles[name] = box


    def set_object_position (self, name, position, parent):

        # DSR (mm) to Coppelia (m)
        position = np.multiply(position, 0.001).tolist()

        object      = self.sim.getObjectHandle(name)
        relative_to = self.sim.getObjectHandle(parent)

        self.sim.setObjectPosition(object, relative_to, position)

    def get_object_pose (self, name, parent='base'):

        if name in self.handles.keys():
            object = self.handles[name]
        else:
            object = self.sim.getObjectHandle (name)
        relative_to = self.sim.getObjectHandle(parent)
        pose = self.sim.getObjectPose (object, relative_to)

        pos = np.multiply(pose[:3], 1000)
        rot = pose[3:]

        return pos, rot


    def set_object_pose(self, name, pose, parent):

        # DSR (mm) to Coppelia (m)
        position = np.multiply(pose[:3], 0.001).tolist()
        
        # Its not clear how to pass orientation
        # TODO migrate to scipy rotation
        # orientation = self.get_quaternion_from_euler (pose[3], pose[4], -pose[5]) # (*pose[3:])

        orientation = R.from_euler('xyz', pose[3:]).as_quat().tolist()
        
        object      = self.handles[name]
        relative_to = self.sim.getObjectHandle(parent)

        self.sim.setObjectPose(object, relative_to, position+orientation)

        # self.sim.callScriptFunction("test_print@"+ self.SCRIPTED_OBJ, 1, object, position+orientation, relative_to)

    def set_multiple_objects_poses (self, names, poses, parent):
        new_poses = []
        object_handles  = []
        for i in range(len(poses)):
            pose  = poses[i]
            position = np.multiply(pose[:3], 0.001).tolist()
            
            # Its not clear how to pass orientation
            # TODO migrate to scipy rotation
            # orientation = self.get_quaternion_from_euler (pose[3], pose[4], -pose[5]) # (*pose[3:])

            # print ("before", np.degrees(pose[3:]))

            orientation  = R.from_euler('xyz', pose[3:]).as_quat().tolist()
            # orientation2 = R.from_euler('XYZ', pose[3:]).as_quat().tolist()


            new_poses.append (position+orientation)
            object_handles.append (self.handles[names[i]])
        
        self.sim.callScriptFunction ("set_multiple_objects@gen3", 1, new_poses, object_handles, parent)


    def set_joints (self, joints):
        
        # for i in range (7):
        # joint_handle = self.sim.getObjectHandle (names[0])
        # sim_pos = self.sim.getJointPosition (joint_handle)
        # print (np.degrees(sim_pos), (joints[0] + 90)%360)
        joints = np.radians(joints).tolist()
        self.sim.callScriptFunction ("set_joints@gen3", 1, joints)

        # print ("-----------------")


    def set_arm_position (self, pose):
        # DSR (mm) to Coppelia (m)
        position = np.multiply(pose[:3], 0.001).tolist()
        
        # Its not clear how to pass orientation
        orientation = self.get_quaternion_from_euler (0, -np.pi/2, 0)

        object      = self.handles['goal']
        relative_to = self.sim.getObjectHandle('base')

        self.sim.setObjectPose(object, relative_to, position+orientation)

    def close_gripper (self):
        self.sim.callScriptFunction ("close@ROBOTIQ_85", 1)

    def open_gripper (self):
        self.sim.callScriptFunction ("open@ROBOTIQ_85", 1)

    def stop_gripper (self):
        self.sim.callScriptFunction ("stop@ROBOTIQ_85", 1)

    def get_gripper_vel (self):

        art_1 = self.sim.getObjectHandle ("ROBOTIQ_85_active1")
        art_2 = self.sim.getObjectHandle ("ROBOTIQ_85_active2")

        j1 = self.sim.getJointVelocity (art_1)
        j2 = self.sim.getJointVelocity (art_2)

        return j1, j2

    def stop_moving (self):
        print ( "----- Stopping simulated arm -----" )
        self.sim.callScriptFunction ("stop_movement@gen3", 1)

    def check_cube_visibility (self):

        # object = self.handles[name]
        # object = self.sim.getObjectHandle ("/lab_table/table")
        # sensor = self.sim.getObjectHandle ("/lab_table/rs_arm")
        # return self.sim.readVisionSensor (sensor)
        # result = self.sim.checkVisionSensor (sensor, object, False)
        # result = self.sim.readVisionSensor (sensor)

        res = self.sim.callScriptFunction ("check_red@/lab_table/rs_arm", 1)

        return res

    def change_color (self, name, color):
        print ("Painting", name, "color", color)
        object = self.handles[name]
        
        # Set it as visible for the camera
        # self.sim.setObjectInt32Param(object, self.sim.objintparam_visibility_layer, 257) 
        # self.sim.setObjectSpecialProperty(object, self.sim.objectspecialproperty_detectable)
        
        self.sim.setShapeColor (object, None, self.sim.colorcomponent_ambient_diffuse, color)

    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]