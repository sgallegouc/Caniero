# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainUI.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from DWidget import DWidget


class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(830, 750)
        guiDlg.setMinimumSize(QSize(830, 750))
        guiDlg.setMaximumSize(QSize(830, 750))
        self.horizontalLayoutWidget = QWidget(guiDlg)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(10, 10, 801, 411))
        self.arriba = QHBoxLayout(self.horizontalLayoutWidget)
        self.arriba.setObjectName(u"arriba")
        self.arriba.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.arriba.setContentsMargins(0, 0, 0, 0)
        self.color = QLabel(self.horizontalLayoutWidget)
        self.color.setObjectName(u"color")
        self.color.setMinimumSize(QSize(400, 400))
        self.color.setMaximumSize(QSize(400, 400))
        self.color.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignTop)

        self.arriba.addWidget(self.color)

        self.widget = DWidget(self.horizontalLayoutWidget)
        self.widget.setObjectName(u"widget")
        self.linea = QFrame(self.widget)
        self.linea.setObjectName(u"linea")
        self.linea.setGeometry(QRect(47, 290, 321, 20))
        self.linea.setFrameShadow(QFrame.Plain)
        self.linea.setLineWidth(5)
        self.linea.setFrameShape(QFrame.HLine)
        self.generar = QPushButton(self.widget)
        self.generar.setObjectName(u"generar")
        self.generar.setGeometry(QRect(60, 340, 300, 25))
        self.generar.setMinimumSize(QSize(300, 0))
        self.generar.setMaximumSize(QSize(300, 16777215))
        self.cubo6 = QLabel(self.widget)
        self.cubo6.setObjectName(u"cubo6")
        self.cubo6.setGeometry(QRect(240, 150, 50, 50))
        self.cubo6.setMinimumSize(QSize(50, 50))
        self.cubo6.setMaximumSize(QSize(50, 50))
        font = QFont()
        font.setPointSize(40)
        font.setBold(True)
        font.setWeight(75)
        self.cubo6.setFont(font)
        self.cubo6.setFrameShape(QFrame.Box)
        self.cubo6.setAlignment(Qt.AlignCenter)
        self.cubo4 = QLabel(self.widget)
        self.cubo4.setObjectName(u"cubo4")
        self.cubo4.setGeometry(QRect(180, 200, 50, 50))
        self.cubo4.setMinimumSize(QSize(50, 50))
        self.cubo4.setMaximumSize(QSize(50, 50))
        self.cubo4.setFont(font)
        self.cubo4.setFrameShape(QFrame.Box)
        self.cubo4.setAlignment(Qt.AlignCenter)
        self.cubo1 = QLabel(self.widget)
        self.cubo1.setObjectName(u"cubo1")
        self.cubo1.setGeometry(QRect(120, 250, 50, 50))
        self.cubo1.setMinimumSize(QSize(50, 50))
        self.cubo1.setMaximumSize(QSize(50, 50))
        self.cubo1.setFont(font)
        self.cubo1.setFrameShape(QFrame.Box)
        self.cubo1.setAlignment(Qt.AlignCenter)
        self.cubo5 = QLabel(self.widget)
        self.cubo5.setObjectName(u"cubo5")
        self.cubo5.setGeometry(QRect(240, 200, 50, 50))
        self.cubo5.setMinimumSize(QSize(50, 50))
        self.cubo5.setMaximumSize(QSize(50, 50))
        self.cubo5.setFont(font)
        self.cubo5.setFrameShape(QFrame.Box)
        self.cubo5.setAlignment(Qt.AlignCenter)
        self.cubo2 = QLabel(self.widget)
        self.cubo2.setObjectName(u"cubo2")
        self.cubo2.setGeometry(QRect(180, 250, 50, 50))
        self.cubo2.setMinimumSize(QSize(50, 50))
        self.cubo2.setMaximumSize(QSize(50, 50))
        self.cubo2.setFont(font)
        self.cubo2.setFrameShape(QFrame.Box)
        self.cubo2.setAlignment(Qt.AlignCenter)
        self.cubo3 = QLabel(self.widget)
        self.cubo3.setObjectName(u"cubo3")
        self.cubo3.setGeometry(QRect(240, 250, 50, 50))
        self.cubo3.setMinimumSize(QSize(50, 50))
        self.cubo3.setMaximumSize(QSize(50, 50))
        self.cubo3.setFont(font)
        self.cubo3.setFrameShape(QFrame.Box)
        self.cubo3.setAlignment(Qt.AlignCenter)
        
        blocks = [self.cubo1, self.cubo2, self.cubo3, self.cubo4, self.cubo5, self.cubo6]
        # blocks = [[1, self.cubo1], [2, self.cubo2], [3, self.cubo3], [4, self.cubo4], [5, self.cubo5], [6, self.cubo6]]
        self.widget.setBlocks(blocks)

        self.arriba.addWidget(self.widget)

        self.verticalLayoutWidget_2 = QWidget(guiDlg)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(10, 420, 802, 321))
        self.abajo = QVBoxLayout(self.verticalLayoutWidget_2)
        self.abajo.setObjectName(u"abajo")
        self.abajo.setContentsMargins(0, 0, 0, 0)
        self.grafic = QLabel(self.verticalLayoutWidget_2)
        self.grafic.setObjectName(u"grafic")
        self.grafic.setMinimumSize(QSize(800, 300))
        self.grafic.setMaximumSize(QSize(800, 300))

        self.abajo.addWidget(self.grafic)

        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"gen3controller", None))
        self.color.setText(QCoreApplication.translate("guiDlg", u"color", None))
        self.generar.setText(QCoreApplication.translate("guiDlg", u"GENERAR ESTADO FINAL", None))
        self.cubo6.setText(QCoreApplication.translate("guiDlg", u"6", None))
        self.cubo4.setText(QCoreApplication.translate("guiDlg", u"4", None))
        self.cubo1.setText(QCoreApplication.translate("guiDlg", u"1", None))
        self.cubo5.setText(QCoreApplication.translate("guiDlg", u"5", None))
        self.cubo2.setText(QCoreApplication.translate("guiDlg", u"2", None))
        self.cubo3.setText(QCoreApplication.translate("guiDlg", u"3", None))
        self.grafic.setText(QCoreApplication.translate("guiDlg", u"grafic", None))
    # retranslateUi