# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'UserInterface.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
import rospy

class Ui_Waiterbot_1(object):
    def setupUi(self, Waiterbot_1):
        Waiterbot_1.setObjectName("Waiterbot_1")
        Waiterbot_1.resize(442, 434)
        self.dockWidgetContents = QtWidgets.QWidget()
        self.dockWidgetContents.setObjectName("dockWidgetContents")
        self.gridLayoutWidget = QtWidgets.QWidget(self.dockWidgetContents)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(0, 0, 221, 271))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(4, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 2, 0, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 1, 0, 1, 1)
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.dockWidgetContents)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(220, 0, 221, 271))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(4, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.pushButton_6 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.pushButton_6.setObjectName("pushButton_6")
        self.gridLayout_2.addWidget(self.pushButton_6, 0, 0, 1, 1)
        self.pushButton_5 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout_2.addWidget(self.pushButton_5, 1, 0, 1, 1)
        self.gridLayoutWidget_3 = QtWidgets.QWidget(self.dockWidgetContents)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(0, 270, 441, 141))
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.pushButton_8 = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        self.pushButton_8.setObjectName("pushButton_8")
        self.gridLayout_3.addWidget(self.pushButton_8, 0, 0, 1, 1)
        Waiterbot_1.setWidget(self.dockWidgetContents)

        self.retranslateUi(Waiterbot_1)
        QtCore.QMetaObject.connectSlotsByName(Waiterbot_1)

    def retranslateUi(self, Waiterbot_1):
        _translate = QtCore.QCoreApplication.translate
        Waiterbot_1.setWindowTitle(_translate("Waiterbot_1", "DockWidget"))
        self.pushButton_2.setText(_translate("Waiterbot_1", "Table 3"))
        self.pushButton_3.setText(_translate("Waiterbot_1", "Table 1"))
        self.pushButton_6.setText(_translate("Waiterbot_1", "Table 2"))
        self.pushButton_5.setText(_translate("Waiterbot_1", "Table 4"))
        self.pushButton_8.setText(_translate("Waiterbot_1", "Kitchen"))

if __name__ == "__main__":
    rospy.init_node("gui",anonymous=False)
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Waiterbot_1 = QtWidgets.QDockWidget()
    ui = Ui_Waiterbot_1()
    ui.setupUi(Waiterbot_1)
    Waiterbot_1.show()
    sys.exit(app.exec_())