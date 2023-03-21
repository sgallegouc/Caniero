#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2023 by YOUR NAME HERE
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
import cv2
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import traceback
import numpy as np
import RoboCompYoloObjects
import Ice
import RoboCompKinovaArm

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)



class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100
        self.state = 'searching'
        if startup_check:
            self.startup_check()
        else:

            try:
                self.yolo_object_names = self.yoloobjects_proxy.getYoloObjectNames()
            except Ice.Exception as e:
                print(str(e) + " Error connecting with YoloObjects interface to retrieve names")


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


    @QtCore.Slot()
    def compute(self):
        #print('SpecificWorker.compute...')
        color, depth, all = self.read_camera("camera_arm")
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        yolo_objects = self.read_yolo(color)

        fin = False
        agarrar = False
        cup = self.select_cup(yolo_objects, color)
        if not fin:
            if cup is not None:
                fin = self.aproximar(cup)
            else:
                print("Cup not seen")
        else:
            print("Objetivo alcanzado")

            # if not agarrar:
            #     agarrar = self.gripper()
            # else:
            #     print("vaso agarrado")



        # Drawing cross on the webcam feed
        width, height, _ = color.shape
        cv2.line(color, (width//2, 0), (width//2, height), (0, 0, 255), 1)
        cv2.line(color, (0, height//2), (width, height//2), (0, 0, 255), 1)
        cv2.imshow("top", color)
        cv2.waitKey(5)

        #draw_objects_on_2dview(objects, RoboCompYoloObjects.TBox())

    def aproximar(self, box):

        print(box.x, box.z)
        # condición de salida
        if abs(box.x) < 10 and abs(box.z) < 10:
            return True

        # Obtener la posición actual del brazo
        try:
            current_pose = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

        print("Box:", box.x, "Hand:", current_pose.x)
        if box.x > 0:
            offset_x = -10
        else:
            offset_x = 10
        if box.z > 0:
            offset_y = -10
        else:
            offset_y = 10

        new_pose = current_pose
        new_pose.x += offset_x
        new_pose.y += offset_y

        # movemos el brazo
        try:
            self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

    def select_cup(self, yolo_objects, frame):
        box = None
        for b in yolo_objects:
            if self.yolo_object_names[b.type] == "cup":
                box = b
                break

        if box is not None:
            cv2.rectangle(frame, (box.left, box.top), (box.right, box.bot), (255, 0, 0), 2)
        return box

    def gripper(self):
        #condicion de salida
        gripper = self.kinovaarm_proxy.getGripperState()
        print (gripper.distance)

    def searching(self):
        pass
       # for r in :
        #    if r[0] == 'vaso':
                # Si encontramos el vaso, retornamos True
         #       return True

        # Si no encontramos el vaso, retornamos False
        return False

    def approaching(self):
        pass

    def catching(self):
        pass


    def read_camera(self, camera_name):
        try:
            all = self.camerargbdsimple_proxy.getAll(camera_name)
            color = np.frombuffer(all.image.image, np.uint8).reshape(all.image.height, all.image.width, all.image.depth)
            depth = np.frombuffer(all.depth.depth, np.float32).reshape(all.depth.height, all.depth.width)
        except Ice.Exception as e:
            traceback.print_exc()
            print(e)
        return color, depth, all

    def connect_to_yolo(self):
        # crear el objeto de proxy
        try:
            proxy = self.stringToProxy("YoloObjects:default -p 10000")
        except Ice.Exception as e:
            print(str(e) + " Error creating proxy")

        # crear el objeto de YoloObjects utilizando el proxy
        yoloobjects_proxy = RoboCompYoloObjects.YoloObjectsPrx.checkedCast(proxy)
        if not yoloobjects_proxy:
            raise RuntimeError("Invalid proxy")
        return yoloobjects_proxy
    def read_yolo(self, frame):
        # get list of object's names from YOLO
        try:
            yolo_objects = self.yoloobjects_proxy.getYoloObjects()
            #print(yolo_objects)
            #pintar
            return yolo_objects.objects
        except Ice.Exception as e:
            print(str(e) + " Error connecting with YoloObjects interface to retrieve names")
        return []

    def draw_objects_on_2dview(self, objects, selected):
        items = []
        for i in items:
            self.viewer.scene.removeItem(i)
        items.clear()

        # draw rest
        for o in objects:
            c = COLORS[o.type]
            color = QtGui.QColor(c[2], c[1], c[0])  # BGR
            item = self.viewer.scene.addRect(-200, -200, 400, 400, QtGui.QPen(color, 20))

            corrected = (self.robot.get_tf_cam_to_base() * np.array([o.x, o.y, o.z, 1.0]))[:2]
            item.setPos(corrected[0], corrected[1])
            items.append(item)
            yolo = self.robot.get_tf_cam_to_base() * np.array([o.x, o.y, o.z, 1.0])
            # print(__FUNCTION__, corrected[0], corrected[1], yolo[0], yolo[1])



        #def read_yolo(self, camera):

    # Carga la configuración y los pesos del modelo YOLO
    #net = darknet.load_net("yolo_cfg_file.cfg", "yolo_weights_file.weights", 0)
    #meta = darknet.load_meta("yolo_data_file.data")

    #while True:
        # Captura la imagen de la cámara
     #   image = camera.getImage()

        # Convierte la imagen de BGR a RGB (necesario para YOLO)
      #  image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Procesa la imagen con el modelo YOLO
       # detections = darknet.detect(net, meta, image)

        # Imprime los objetos detectados en la imagen
        #for detection in detections:
         #   print("Se detectó un objeto:", detection[0].decode())

        # Muestra la imagen con los cuadros delimitadores de los objetos detectados
        #image = darknet.draw_boxes(detections, image, meta)
        #cv2.imshow("YOLO Object Detection", image)

        # Espera a que se presione una tecla para salir
        #if cv2.waitKey(1) & 0xFF == ord('q'):
         #   break

    # Libera los recursos utilizados
    #camera.release()
    #cv2.destroyAllWindows()

    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.Point3D from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.Point3D()
        print(f"Testing RoboCompCameraRGBDSimple.TPoints from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TPoints()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompCoppeliaUtils.PoseType from ifaces.RoboCompCoppeliaUtils")
        test = ifaces.RoboCompCoppeliaUtils.PoseType()
        print(f"Testing RoboCompCoppeliaUtils.SpeedType from ifaces.RoboCompCoppeliaUtils")
        test = ifaces.RoboCompCoppeliaUtils.SpeedType()
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        QTimer.singleShot(200, QApplication.instance().quit)


    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)
    # self.camerargbdsimple_proxy.getPoints(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.Point3D
    # RoboCompCameraRGBDSimple.TPoints
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompCoppeliaUtils you can call this methods:
    # self.coppeliautils_proxy.addOrModifyDummy(...)
    # self.coppeliautils_proxy.setDummySpeed(...)

    ######################
    # From the RoboCompCoppeliaUtils you can use this types:
    # RoboCompCoppeliaUtils.PoseType
    # RoboCompCoppeliaUtils.SpeedType

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm_proxy.closeGripper(...)
    # self.kinovaarm_proxy.getCenterOfTool(...)
    # self.kinovaarm_proxy.getGripperState(...)
    # self.kinovaarm_proxy.openGripper(...)
    # self.kinovaarm_proxy.setCenterOfTool(...)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # RoboCompKinovaArm.TPose
    # RoboCompKinovaArm.TGripper


