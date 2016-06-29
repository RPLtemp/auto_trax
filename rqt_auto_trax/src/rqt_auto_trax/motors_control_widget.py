#!/usr/bin/env python
import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

class MotorsControlWidget(QtGui.QWidget):
	def __init__(self):
		super(MotorsControlWidget, self).__init__()
		self.setObjectName('MotorsControlWidget')

		# Load UI
		ui_file = os.path.join(rospkg.RosPack().get_path('rqt_auto_trax'), 'resource', 'motors_control_widget.ui')
		loadUi(ui_file, self)