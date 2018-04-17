#!/usr/bin/env python

PKG = 'wsg50_driver'
NAME = 'autotest'

import sys
import json                 # Json API
import rospy                # ROS API
import rostest				# ROS specific testing library
import unittest				# Python generic testing library used by ROS
from std_srvs.srv import *
from wsg50_common.srv import *     # For loading service


class TestGripper(unittest.TestCase):
	# Each test case is individually defined as a function

	# Test a large program
	def test_autotest(self):
		rospy.wait_for_service('/wsg50_driver/homing')
		rospy.wait_for_service('/wsg50_driver/move')
		srv_home = rospy.ServiceProxy('/wsg50_driver/homing', Empty)
		srv_move = rospy.ServiceProxy('/wsg50_driver/move', Move)
		srv_home()
		srv_move(0, 10)

	# --------------------------------------------
	# Extend it with new tests here
	# Please create programs in testing_tools.py file to keep this .py small
	# --------------------------------------------

if __name__ == '__main__':
	rospy.init_node('wsg50_driver_autotest') # Needed for calling the action, without init it fails
	rostest.rosrun(PKG, NAME, TestGripper, sys.argv)