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
from PIL.ImageChops import offset
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
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)



class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100
        self.state = 'searching'
        self.estadoMaquina = "encarar"
        self.aux_pose = 0
        self.tEspera = 0
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

        cup = self.select_cup(yolo_objects, color)

        # Obtener la posición actual del brazo
        try:
            self.current_pose = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

        try:
            self.kinovaarm_proxy.stopGripper()
        except Ice.Exception as e:
            print(str(e) + " Error stop")

        if self.estadoMaquina == "encarar":
            if cup is not None:
                self.encarar(cup)
        elif self.estadoMaquina == "aproximar":
            if cup is not None:
                self.aproximar(cup)
        elif self.estadoMaquina == "inicioUltimoEmpujon":
            self.inicioUltimoEmpujon()
        elif self.estadoMaquina == "ultimoEmpujon":
            self.ultimoEmpujon()
        elif self.estadoMaquina == "acabarUltimoEmpujon":
            self.acabarUltimoEmpujon()
        elif self.estadoMaquina == "empezarGripper":
            self.empezarGripper()
        # elif cup is not None:
        #     self.estadoMaquina = "encarar"
        elif self.estadoMaquina == "acabarGripper":
            self.acabarGripper()
        elif self.estadoMaquina == "empezarDepositar":
            self.empezarDepositar()
        elif self.estadoMaquina == "acabarDepositar":
            self.acabarDepositar()
        elif self.estadoMaquina == "entregarVaso":
            self.entregarVaso()
        elif self.estadoMaquina == "iniciarEspera":
            self.iniciarEspera()
        elif self.estadoMaquina == "acabarEspera":
            self.acabarEspera()
        elif self.estadoMaquina == "servirVaso":
            self.servirVaso()
        elif self.estadoMaquina == "devolver":
            self.devolver()
        elif self.estadoMaquina == "acabado":
            self.acabado()
        else:
            print("Objetivo alcanzado")

        # Drawing cross on the webcam feed
        width, height, _ = color.shape
        cv2.line(color, (width//2, 0), (width//2, height), (0, 0, 255), 1)
        cv2.line(color, (0, height//2), (width, height//2), (0, 0, 255), 1)
        cv2.imshow("top", color)
        cv2.waitKey(5)

        # draw_objects_on_2dview(objects, RoboCompYoloObjects.TBox())

    def encarar(self, box):
        # print(box.x, box.z)
        # condición de salida
        if abs(box.x) < 2 and abs(box.z) < 25:
            self.estadoMaquina = "aproximar"

        # print("Box--x:", box.x, "Hand:", current_pos e.x)
        # print("Box--z:", box.z, "Hand:", current_pose.z)

        new_pose = self.current_pose
        if abs(box.x) > 2:
            if box.x > 0:
                offset_x = -2
            else:
                offset_x = 2
            new_pose.x += offset_x
        if abs(box.z) > 25:
            if box.z > 0:
                offset_z = -2
            else:
                offset_z = 2
            new_pose.z += offset_z

        # movemos el brazo
        try:
            self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

    def aproximar(self,box):
        # condición de salida
        if abs(box.y) < 190:
            self.estadoMaquina = "inicioUltimoEmpujon"

        if abs(box.x) >= 2 or abs(box.z) >= 25:
            self.estadoMaquina = "encarar"

        print("Box--y:", box.y, "Hand:", self.current_pose.y)
        new_pose = self.current_pose
        if abs(box.y) > 0:
            offset_y = -10
        else:
            offset_y = 10

        new_pose.y += offset_y

        # movemos el brazo
        try:
            self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

    def inicioUltimoEmpujon(self):
        new_pose = self.current_pose
        self.aux_pose = self.current_pose.y
        new_pose.y += -90
        self.estadoMaquina = "ultimoEmpujon"

        try:
            self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

    def ultimoEmpujon(self):
        if abs(self.aux_pose - 79) > abs(self.current_pose.y) < abs(self.aux_pose - 81):
            self.tEspera = time.time()
            self.estadoMaquina = "acabarUltimoEmpujon"
        else:
            try:
                self.kinovaarm_proxy.setCenterOfTool(self.aux_pose - 90, RoboCompKinovaArm.ArmJoints.base)
            except Ice.Exception as e:
                print(str(e) + " Error connecting with Kinova Arm")

    def acabarUltimoEmpujon(self):
        tActual = time.time() - 1 - self.tEspera
        if tActual > 2:
            self.estadoMaquina = "empezarGripper"

    def empezarGripper(self):
        gripper = self.kinovaarm_proxy.getGripperState()
        self.kinovaarm_proxy.closeGripper()
        print("gripper distance: " + str(gripper.opening))

        if gripper.opening > 190:
            self.kinovaarm_proxy.stopGripper()
            self.aux_pose = self.current_pose.z
            self.estadoMaquina = "acabarGripper"

    def acabarGripper(self):
        new_pose = self.current_pose
        new_pose.z += 75

        if abs(self.aux_pose + 75) < abs(self.current_pose.z):
            self.estadoMaquina = "empezarDepositar"
        else:
            try:
                self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
            except Ice.Exception as e:
                print(str(e) + " Error connecting with Kinova Arm")

    def empezarDepositar(self):
        new_pose = self.current_pose
        self.aux_pose = self.current_pose.y
        new_pose.y += 150
        self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
        self.estadoMaquina = "acabarDepositar"

    def acabarDepositar(self):
        print(self.aux_pose)
        print(self.current_pose.y)
        if (self.aux_pose + 150) < self.current_pose.y:
            self.estadoMaquina = "entregarVaso"
        else:
            try:
                new_pose = self.current_pose
                new_pose.y += 20
                self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
            except Ice.Exception as e:
                print(str(e) + " Error connecting with Kinova Arm")

    def entregarVaso(self):
        try:
            new_pose = self.current_pose
            new_pose.x = -50
            new_pose.y = -515
            new_pose.z = 80
            self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
            if self.current_pose.x==-50 and self.current_pose.y==-515 and self.current_pose.z==80:
                self.estadoMaquina = "iniciarEspera"

        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

    def iniciarEspera(self):
        try:
            self.tEspera = time.time()
            self.estadoMaquina = "acabarEspera"

        except Ice.Exception as e:
            print(str(e) + "Error en el timer")

    def acabarEspera(self):
        try:
            tActual = time.time() - 1 - self.tEspera
            if tActual > 5:
                self.estadoMaquina = "servirVaso"
        except Ice.Exception as e:
            print(str(e) + "Error en el timer")

    def servirVaso(self):
        try:
            new_pose = self.current_pose
            new_pose.x = 400
            new_pose.y = -600
            new_pose.z = 92
            self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
            if self.current_pose.x == 400 and self.current_pose.y == -600 and self.current_pose.z == 92:
                self.estadoMaquina = "devolver"

        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

    def devolver(self):
        try:
            new_pose = self.current_pose
            new_pose.x = 400
            new_pose.y = -600
            new_pose.z = 20
            self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
            self.tEspera = time.time()
            self.estadoMaquina = "acabado"
        except Ice.Exception as e:
            print(str(e) + " Error connecting with Kinova Arm")

    def acabado(self):
        try:
            new_pose = self.current_pose
            new_pose.y = -400
            tActual = time.time() - 1 - self.tEspera
            if tActual > 3:
                self.kinovaarm_proxy.openGripper()
                if self.current_pose.x > 398 and abs(self.current_pose.y) > 398 and self.current_pose.z < 21:
                    self.kinovaarm_proxy.setCenterOfTool(new_pose, RoboCompKinovaArm.ArmJoints.base)
                    self.estadoMaquina = "fin"
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

    def read_camera(self, camera_name):
        color = []
        depth = []
        all = []
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


