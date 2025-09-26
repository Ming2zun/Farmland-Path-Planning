#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
#                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


################################################################################
## Form generated from reading UI file 'nav2.ui'
##
## Created by: Qt User Interface Compiler version 6.6.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QVBoxLayout, QWidget)

class Ui_navigation(object):
    def setupUi(self, navigation):
        if not navigation.objectName():
            navigation.setObjectName(u"navigation")
        navigation.resize(316, 767)
        self.pb_leading = QPushButton(navigation)
        self.pb_leading.setObjectName(u"pb_leading")
        self.pb_leading.setGeometry(QRect(110, 210, 89, 25))
        self.label_2 = QLabel(navigation)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(130, 30, 67, 17))
        self.label_10 = QLabel(navigation)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(120, 480, 141, 20))
        self.label_11 = QLabel(navigation)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setGeometry(QRect(140, 260, 67, 17))
        self.layoutWidget = QWidget(navigation)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(30, 170, 271, 27))
        self.horizontalLayout_6 = QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.l_wide = QLineEdit(self.layoutWidget)
        self.l_wide.setObjectName(u"l_wide")

        self.horizontalLayout_6.addWidget(self.l_wide)

        self.label_4 = QLabel(self.layoutWidget)
        self.label_4.setObjectName(u"label_4")

        self.horizontalLayout_6.addWidget(self.label_4)

        self.l_rad = QLineEdit(self.layoutWidget)
        self.l_rad.setObjectName(u"l_rad")

        self.horizontalLayout_6.addWidget(self.l_rad)

        self.label_5 = QLabel(self.layoutWidget)
        self.label_5.setObjectName(u"label_5")

        self.horizontalLayout_6.addWidget(self.label_5)

        self.layoutWidget1 = QWidget(navigation)
        self.layoutWidget1.setObjectName(u"layoutWidget1")
        self.layoutWidget1.setGeometry(QRect(30, 140, 281, 19))
        self.horizontalLayout_8 = QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.label_7 = QLabel(self.layoutWidget1)
        self.label_7.setObjectName(u"label_7")

        self.horizontalLayout_8.addWidget(self.label_7)

        self.label_9 = QLabel(self.layoutWidget1)
        self.label_9.setObjectName(u"label_9")

        self.horizontalLayout_8.addWidget(self.label_9)

        self.layoutWidget2 = QWidget(navigation)
        self.layoutWidget2.setObjectName(u"layoutWidget2")
        self.layoutWidget2.setGeometry(QRect(30, 100, 261, 27))
        self.horizontalLayout_5 = QHBoxLayout(self.layoutWidget2)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.l_speed = QLineEdit(self.layoutWidget2)
        self.l_speed.setObjectName(u"l_speed")

        self.horizontalLayout_5.addWidget(self.l_speed)

        self.label = QLabel(self.layoutWidget2)
        self.label.setObjectName(u"label")

        self.horizontalLayout_5.addWidget(self.label)

        self.l_angle = QLineEdit(self.layoutWidget2)
        self.l_angle.setObjectName(u"l_angle")

        self.horizontalLayout_5.addWidget(self.l_angle)

        self.label_3 = QLabel(self.layoutWidget2)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_5.addWidget(self.label_3)

        self.layoutWidget3 = QWidget(navigation)
        self.layoutWidget3.setObjectName(u"layoutWidget3")
        self.layoutWidget3.setGeometry(QRect(50, 70, 251, 19))
        self.horizontalLayout_7 = QHBoxLayout(self.layoutWidget3)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.label_6 = QLabel(self.layoutWidget3)
        self.label_6.setObjectName(u"label_6")

        self.horizontalLayout_7.addWidget(self.label_6)

        self.label_8 = QLabel(self.layoutWidget3)
        self.label_8.setObjectName(u"label_8")

        self.horizontalLayout_7.addWidget(self.label_8)

        self.layoutWidget4 = QWidget(navigation)
        self.layoutWidget4.setObjectName(u"layoutWidget4")
        self.layoutWidget4.setGeometry(QRect(30, 520, 251, 60))
        self.verticalLayout_3 = QVBoxLayout(self.layoutWidget4)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.pb_stop_na = QPushButton(self.layoutWidget4)
        self.pb_stop_na.setObjectName(u"pb_stop_na")

        self.horizontalLayout_9.addWidget(self.pb_stop_na)

        self.pb_na = QPushButton(self.layoutWidget4)
        self.pb_na.setObjectName(u"pb_na")

        self.horizontalLayout_9.addWidget(self.pb_na)


        self.verticalLayout_3.addLayout(self.horizontalLayout_9)

        self.pb_save_na = QPushButton(self.layoutWidget4)
        self.pb_save_na.setObjectName(u"pb_save_na")

        self.verticalLayout_3.addWidget(self.pb_save_na)

        self.layoutWidget5 = QWidget(navigation)
        self.layoutWidget5.setObjectName(u"layoutWidget5")
        self.layoutWidget5.setGeometry(QRect(20, 300, 279, 60))
        self.horizontalLayout = QHBoxLayout(self.layoutWidget5)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.L_yaml = QLineEdit(self.layoutWidget5)
        self.L_yaml.setObjectName(u"L_yaml")

        self.horizontalLayout.addWidget(self.L_yaml)

        self.pb_choose = QPushButton(self.layoutWidget5)
        self.pb_choose.setObjectName(u"pb_choose")

        self.horizontalLayout.addWidget(self.pb_choose)

        self.label_12 = QLabel(navigation)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setGeometry(QRect(130, 370, 141, 20))
        self.L_save = QLineEdit(navigation)
        self.L_save.setObjectName(u"L_save")
        self.L_save.setGeometry(QRect(30, 610, 271, 121))
        self.layoutWidget6 = QWidget(navigation)
        self.layoutWidget6.setObjectName(u"layoutWidget6")
        self.layoutWidget6.setGeometry(QRect(20, 400, 271, 62))
        self.verticalLayout = QVBoxLayout(self.layoutWidget6)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.pb_open_working = QPushButton(self.layoutWidget6)
        self.pb_open_working.setObjectName(u"pb_open_working")

        self.horizontalLayout_2.addWidget(self.pb_open_working)

        self.pb_stop_working = QPushButton(self.layoutWidget6)
        self.pb_stop_working.setObjectName(u"pb_stop_working")

        self.horizontalLayout_2.addWidget(self.pb_stop_working)


        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.lineEdit_6 = QLineEdit(self.layoutWidget6)
        self.lineEdit_6.setObjectName(u"lineEdit_6")

        self.horizontalLayout_3.addWidget(self.lineEdit_6)

        self.pb_import = QPushButton(self.layoutWidget6)
        self.pb_import.setObjectName(u"pb_import")

        self.horizontalLayout_3.addWidget(self.pb_import)


        self.verticalLayout.addLayout(self.horizontalLayout_3)


        self.retranslateUi(navigation)

        QMetaObject.connectSlotsByName(navigation)
    # setupUi

    def retranslateUi(self, navigation):
        navigation.setWindowTitle(QCoreApplication.translate("navigation", u"navigation", None))
        self.pb_leading.setText(QCoreApplication.translate("navigation", u"Leading-in", None))
        self.label_2.setText(QCoreApplication.translate("navigation", u"Config", None))
        self.label_10.setText(QCoreApplication.translate("navigation", u"Navigation", None))
        self.label_11.setText(QCoreApplication.translate("navigation", u"Rote", None))
        self.l_wide.setText(QCoreApplication.translate("navigation", u"12.0", None))
        self.label_4.setText(QCoreApplication.translate("navigation", u"m", None))
        self.l_rad.setText(QCoreApplication.translate("navigation", u"3.2", None))
        self.label_5.setText(QCoreApplication.translate("navigation", u"m", None))
        self.label_7.setText(QCoreApplication.translate("navigation", u"Working width", None))
        self.label_9.setText(QCoreApplication.translate("navigation", u"Minimum radius", None))
        self.l_speed.setText(QCoreApplication.translate("navigation", u"1.0", None))
        self.label.setText(QCoreApplication.translate("navigation", u"m/s", None))
        self.l_angle.setText(QCoreApplication.translate("navigation", u"25.0", None))
        self.label_3.setText(QCoreApplication.translate("navigation", u"\u00b0 ", None))
        self.label_6.setText(QCoreApplication.translate("navigation", u"Speed", None))
        self.label_8.setText(QCoreApplication.translate("navigation", u"Max angle", None))
        self.pb_stop_na.setText(QCoreApplication.translate("navigation", u"Stop", None))
        self.pb_na.setText(QCoreApplication.translate("navigation", u"Nav", None))
        self.pb_save_na.setText(QCoreApplication.translate("navigation", u"Save", None))
        self.L_yaml.setText(QCoreApplication.translate("navigation", u"A.yaml", None))
        self.pb_choose.setText(QCoreApplication.translate("navigation", u"Choose", None))
        self.label_12.setText(QCoreApplication.translate("navigation", u"Working", None))
        self.L_save.setText("")
        self.pb_open_working.setText(QCoreApplication.translate("navigation", u"Open", None))
        self.pb_stop_working.setText(QCoreApplication.translate("navigation", u"Stop", None))
        self.lineEdit_6.setText(QCoreApplication.translate("navigation", u"A_work_commder.yaml", None))
        self.pb_import.setText(QCoreApplication.translate("navigation", u"Import", None))
    # retranslateUi

