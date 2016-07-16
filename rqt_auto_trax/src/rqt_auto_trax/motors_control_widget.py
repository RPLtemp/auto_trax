#!/usr/bin/env python
import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from auto_trax_msgs.srv import IOSetpoint

class MotorsControlWidget(QtGui.QWidget):
	# String constants
	STR_APPLY_MOTOR_SPEED_SERVICE_NAME = 'auto_trax_io/apply_motor_speed'
	STR_APPLY_STEERING_ANGLE_SERVICE_NAME = 'auto_trax_io/apply_steering_angle'

	def __init__(self):
		super(MotorsControlWidget, self).__init__()
		self.setObjectName('MotorsControlWidget')

		# Load UI
		ui_file = os.path.join(rospkg.RosPack().get_path('rqt_auto_trax'), 'resource', 'motors_control_widget.ui')
		loadUi(ui_file, self)

		# Create ROS service proxies
		self.set_motor_speed = rospy.ServiceProxy(self.STR_APPLY_MOTOR_SPEED_SERVICE_NAME, IOSetpoint)
		self.set_steering_angle = rospy.ServiceProxy(self.STR_APPLY_STEERING_ANGLE_SERVICE_NAME, IOSetpoint)

		# Set the functions that are called when signals are emitted
		self.set_motor_speed_button.pressed.connect(self.set_motor_speed_button_pressed)
		self.stop_motor_button.pressed.connect(self.stop_motor_button_pressed)

	def set_motor_speed_button_pressed(self):
		speed = self.motor_speed_spin_box.value()
		self.set_motor_speed(speed)

	def stop_motor_button_pressed(self):
		self.set_motor_speed(0)
		self.set_steering_angle(0)