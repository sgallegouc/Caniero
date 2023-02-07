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

import multiprocessing as mp
import time
import vid_streamv32 as vs
import cv2
import numpy as np

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 0

        #Current Cam
        self.camProcess = None
        self.cam_queue = None
        self.stopbit = None
        self.camlink = 'rtsp://192.168.1.10/depth' # Add your RTSP cam link
        self.framerate = 15

        #Current Cam
        self.colorCamProcess = None
        self.color_queue = None
        self.colorStopbit = None
        self.colorLink = 'rtsp://192.168.1.10/color' # Add your RTSP cam link
        self.colorFramerate = 15

        #set  queue size
        self.cam_queue = mp.Queue(maxsize=1)
        self.color_queue = mp.Queue(maxsize=1)

        #get all cams
        time.sleep(3)

        self.stopbit = mp.Event()
        self.camProcess = vs.StreamCapture( self.camlink,
                                            self.stopbit,
                                            self.cam_queue,
                                            self.framerate)

                            
        self.camProcess.start()
        
        self.colorStopbit = mp.Event()
        self.colorCamProcess = vs.StreamCapture(self.colorLink,
                                                self.colorStopbit,
                                                self.color_queue,
                                                self.colorFramerate)
        self.colorCamProcess.start()


        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""
        if self.stopbit is not None:
            self.stopbit.set()
            while not self.cam_queue.empty():
                try:
                    _ = self.cam_queue.get()
                except:
                    break
                self.cam_queue.close()

            self.camProcess.join()

        if self.colorStopbit is not None:
            self.colorStopbit.set()
            while not self.color_queue.empty():
                try:
                    _ = self.color_queue.get()
                except:
                    break
                self.color_queue.close()

            self.colorCamProcess.join()


        

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        # print('SpecificWorker.compute...')
        
        if not self.cam_queue.empty():
            # print('Got frame')
            cmd, val = self.cam_queue.get()

            # if cmd == vs.StreamCommands.RESOLUTION:
            #     pass #print(val)

            if cmd == vs.StreamCommands.FRAME:
                if val is not None:
                    
                    print ("depth: ", val[240][127])

                    val = val.astype(np.uint8)

                    color = cv2.cvtColor(val, cv2.COLOR_GRAY2RGB)
                    qt_color = QImage(color, val.shape[1], val.shape[0], QImage.Format_RGB888)
                    pix_color = QPixmap.fromImage(qt_color).scaled(self.ui.depth.width(), self.ui.depth.height())
                    self.ui.depth.setPixmap(pix_color)

                    # print (val.shape, val)


        if not self.color_queue.empty():
            # print('Got frame')
            color_cmd, color_val = self.color_queue.get()

            # if cmd == vs.StreamCommands.RESOLUTION:
            #     pass #print(val)

            if color_cmd == vs.StreamCommands.FRAME:
                if color_val is not None:
                    
                    # print ("img: ", val.shape[1], val.shape[0], " UI: ", self.ui.color.width(), self.ui.color.height() )

                    color_color = cv2.cvtColor(color_val, cv2.COLOR_BGR2RGB)
                    color_qt_color = QImage(color_color, color_val.shape[1], color_val.shape[0], QImage.Format_RGB888)
                    color_pix_color = QPixmap.fromImage(color_qt_color).scaled(self.ui.color.width(), self.ui.color.height())
                    self.ui.color.setPixmap(color_pix_color)

                    # print (val.shape, val)

        return True

    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getAll method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getAll(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TDepth()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getImage(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TImage()
        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD


