#!/usr/bin/env python
# -*- coding: utf-8 -*-

# drag and bot GmbH 2019
# This driver uses the new GCL protocol which is a Telnet-based protocol, much easier
# Interface documentation: https://schunk.com/fileadmin/pim/docs/IM0022534.PDF

# TODO:
# - socket reconnection

import re 			# Regular expresions for parsing commands
import sys			# For program exiting
import threading	# Events to notify when a command finishes with error or no error
import thread 		# To run new threads
import socket 		# Socket communication
import rospy		# ROS api
import time

# ROS Communication messages
from dnb_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from wsg50_common.msg import *
from wsg50_common.srv import *

debug = False 			# Extended verbose
debug_states = False 	# Show gripper states changes
autorelease = True 		# Execute release automatically before each grasp
rate_hz = 50 			# Hz for publishing information and autosend from gripper
timeout_wait = 60 		# To consider a command as fail, very slow motions can timeout it
fast_stop = True 		# Use FASTSTOP command instead of STOP, which leads the gripper to error. It works better with FASTSTOP

# -----------------------------------------------------------------------------
# Data model containing current Gripper state
# -----------------------------------------------------------------------------
class GripperState():
	position = None # Current position of the gripper in mm
	speed    = None # Speed of the gripper in mm/s
	state    = None # 0..7: Idle 0, Grasping 1, no part 2, part lost 3, holding 4, releasing 5, positioning 6, error 7
	force    = None # Current force of the gripper

	target_force = None			# Because we are using an older interface, first force has to be set as default value for next move, grasp etc. calls
	target_acceleration = None 	# Same for acceleration

	component_status_error = False

	event_fin = threading.Event() 		# Is set after receiving FIN answer (end of command)
	event_no_error = threading.Event() 	# Is set when there is no error (this solves a race condition if robot is alreay in error when starting a command)
	event_err = threading.Event() 		# Set when error answer (ERR) comes instead of FIN. Needs to be manually reset

	# Return True or False if gripper is in error
	# This function only considers state for the evaluation, not the event_err flag
	def is_error(self):
		return self.state == 7 or self.event_err.is_set()	

	# Returns 1 if error, 0 if not
	def get_error_code(self):
		if self.is_error():
			return 1
		else:
			return 0

# -----------------------------------------------------------------------------
# Socket communication class
# -----------------------------------------------------------------------------
class ControlComm():
	socklock = threading.Lock()	# socket lock
	state = GripperState()		# Current gripper state
	spinner = None				# Spinner function (main loop). Needed to have just one running thread
	spinner_info = None			# Spinner function for publishing Ros information
	interface = None			# ROS accesor object

	# Connection function, establish a connection with the gripper
	@classmethod
	def connect(cls):
		ip = rospy.get_param('~ip', "192.168.0.20" ) # IP and port from ROS parameters
		port = int(rospy.get_param('~port', 1000))
		rospy.loginfo("WSG Driver: connecting with " + ip + ":" + str(port))
		try:
			with cls.socklock:
				cls.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				cls.sock.settimeout(1) # 1 Second to consider connection lost
				cls.sock.connect((ip, port))

			# HERE SET COMPONENT STATUS AS OK -> state.component_error == TRUE
			cls.state.component_status_error = False

			# Start spinner only once
			if not cls.spinner:
				cls.spinner = thread.start_new_thread(cls.spin, ())
			if not cls.spinner_info: 
				cls.spinner_info = thread.start_new_thread(cls.spin_info, ())
			rospy.loginfo("WSG Driver: connected to " + ip + ":" + str(port))
			return True
		except Exception as e:
			rospy.logerr("WSG driver error: " + str(e))
			cls.sock = None
			rospy.loginfo("WSG Driver: connection error.")
			return False

	# Asynchronous main loop
	@classmethod
	def spin(cls):
		while not rospy.is_shutdown():

			try:
				packet = cls.sock.recv(4096)				# Wait form a new TCP packet, it can contain more than 1 command
				for packet in str(packet).splitlines(True): # Divide lines for individual commands

					if debug: rospy.logerr("recv: " + str(packet))

					# ---------------------------------------------------------------------------------
					# Answers received, using regex (regular expresions)
					# ---------------------------------------------------------------------------------
					r, v = cls.parse_command(packet, "@POS=float") 		# Asynchronous update from pose
					if r: cls.state.position = float(v[0])
					# ---------------------------------------------------------------------------------
					r, v = cls.parse_command(packet, "@SPEED=float") 	# Asynchronous update from speed
					if r: cls.state.speed = float(v[0])
					# ---------------------------------------------------------------------------------
					r, v = cls.parse_command(packet, "@FORCE=float")	# Asynchronous update from force
					if r: cls.state.force = float(v[0])
					# ---------------------------------------------------------------------------------
					r, v = cls.parse_command(packet, "@GRIPSTATE=int")	# Asynchronous update from gripper state
					if r: 
						if debug_states: 
							if int(v[0]) != cls.state.state: rospy.logerr(cls.state.state) # Print state changes
						cls.state.state = int(v[0])
					# ---------------------------------------------------------------------------------
					r, v = cls.parse_command(packet, "FIN.*") # Regular expression to indicate the correct end of any command
					if r: cls.state.event_fin.set()
					# ---------------------------------------------------------------------------------
					r, v = cls.parse_command(packet, "ERR.*") # Regular expression to indicate the fail end of any command
					if r:
						cls.state.event_err.set()	# Here the order is very important, first mark error
						cls.state.event_fin.set()	# and then mark the command as finished
					# ---------------------------------------------------------------------------------

					if cls.state.is_error():
						if debug: rospy.logerr("WSG driver: gripper achieved error state")
						cls.state.event_fin.set()
					else:
						cls.state.event_no_error.set()
					
			#except socket.timeout:
			#	pass # Timeout is consider no error, because we don't want infinite wait at recv --> FOR CM
			except Exception as e:
				rospy.logerr("WSG driver error: " + str(e)) # TODO: reconnect here

				# HERE SET COMPONENT STATUS AS ERROR -> state.component_error == TRUE
				cls.state.component_status_error = True

				cls.connect() # Try to stablish the connection again

	@classmethod
	def spin_info(cls):
		rate = rospy.Rate(rate_hz)
		while not rospy.is_shutdown():
			# Publish ROS d&b component status topic
			if cls.interface: cls.interface.publish_component_status(cls.state.component_status_error)
			if cls.interface: cls.interface.publish_gripper_status(cls.state)
			rate.sleep()


	# Sends a command to the gripper without waiting for any response
	# Command is a string command
	# Returns True if command was correctly send or False in case of communication error
	@classmethod
	def send_without_response(cls, command):
		with cls.socklock:
			try:
				if debug: rospy.logerr("sent: " + command)
				cls.sock.send((command + "\n").encode('ascii'))
				return True
			except Exception as e:
				rospy.logerr("WSG driver error: " + str(e))
				cls.connect() # try to stablish the connection again
				return False

	# This class sends the command and waits until the given answer is received or error
	# Returns True if answer is received, False if error
	@classmethod
	def send_and_wait_for_fin(cls, command):
		cls.state.event_fin.clear()
		cls.state.event_err.clear()
		if not cls.send_without_response(command): return False
		if not cls.state.event_fin.wait(timeout_wait): return False
		in_error = cls.state.is_error()
		cls.state.event_err.clear()
		return not in_error

	# Needed to publish component status, and in the future pose, speed, force, etc.
	@classmethod
	def set_interface(cls, interface):
		cls.interface = interface

	# Returns True / False and a list with the pattern found. A packet should be a line (one command)
	# https://en.wikipedia.org/wiki/Regular_expression
	@staticmethod
	def parse_command(packet, pattern):
		# Adding .* at the beginning and at the end forces to found the pattern only once (the newest message)
		pattern = pattern.replace("float", "([-+]?\\d*\\.?\\d+)")
		pattern = pattern.replace("string","([^\]\[]*)")
		pattern = pattern.replace("int","([-+]?\\d+)")

		found = re.match(pattern, packet)
		if found:
			return True, list(found.groups())
		else:
			return False, []

# -----------------------------------------------------------------------------
# ROS Interface: for retrocompatibility the existing services are kept.
# This new text protocol brings new functionalities which requires future
# interface use (e.g. grasp with force, no need for set_force service)
# -----------------------------------------------------------------------------
class Interface:
	def __init__(self):
		#self.pub_joint_states     = rospy.Publisher('joint_states', JointState, queue_size = 1)
		self.srv_ack 				= rospy.Service('~ack', 				Empty, 	self.callback_ack)
		self.srv_grasp 				= rospy.Service('~grasp', 				Move, 	self.callback_grasp)
		self.srv_release 			= rospy.Service('~release', 			Move, 	self.callback_release)
		self.srv_move 				= rospy.Service('~move', 				Move, 	self.callback_move)
		self.srv_move_incrementally = rospy.Service('~move_incrementally', 	Incr, 	self.callback_move_incrementally)
		self.srv_homing 			= rospy.Service('~homing', 				Empty, 	self.callback_homing)
		self.srv_stop 				= rospy.Service('~stop', 				Empty, 	self.callback_stop)
		self.srv_set_acceleration 	= rospy.Service('~set_acceleration', 	Conf, 	self.callback_set_acceleration)
		self.srv_set_force 			= rospy.Service('~set_force', 			Conf, 	self.callback_set_force)
		self.pub_component_status 	= rospy.Publisher('~component/status', 	ComponentStatus, queue_size=1, latch=True) # d&b component status topic
		self.pub_gripper_status 	= rospy.Publisher('~status',			Status,	queue_size=1)

	# Error quitting
	# This function resets any error. It returns true if error was able to be cleared
	# There is an small race condition here. If there is no error active, but suddently during FSACK an error comes, then the
	# function can finish without clearing the error. We assume an error can only be produced by a command, therefore no
	# ack should be call during a gripper move. Anyway, in this case, the gripper will stop and produce error by trying
	# to execute the command
	def reset_error(self):
		if ControlComm.state.is_error():
			ControlComm.state.event_no_error.clear()
			ControlComm.send_without_response("FSACK()")
			return ControlComm.state.event_no_error.wait(1) # If after 1 seconds there is no answer, error and return False

	# Error quitting service callback. Interface is ROS message
	def callback_ack(self, req = None):
		self.reset_error()
		return EmptyResponse()

	# Grip. Interface is ROS message
	def callback_grasp(self, req):

		# Autorelease just in case something was gripped before
		if ControlComm.state.position:
			if autorelease: self.callback_release(MoveRequest(ControlComm.state.position,req.speed))
		else:
			rospy.logerr("WSG driver: no prior position information. Check gripper connection")
			return MoveResponse(1)

		self.reset_error()

		if ControlComm.state.target_force:
			if ControlComm.send_and_wait_for_fin("GRIP(" + str(ControlComm.state.target_force) + "," + str(req.width) + "," + str(req.speed) + ")"):
				return MoveResponse(ControlComm.state.get_error_code())
			else:
				# err -> produces False if part not gripped
				return MoveResponse(1)
		else:
			rospy.logerr("WSG driver: attemping to grasp without first providing force")
			return MoveResponse(1)

	# Release gripper, a part should be gripped before. Interface is ROS message
	def callback_release(self, req):
		self.reset_error()
		if ControlComm.state.position:
			distance = req.width - ControlComm.state.position
			if ControlComm.send_and_wait_for_fin("RELEASE(" + str(distance) + "," + str(req.speed) + ")"):
				return MoveResponse(ControlComm.state.get_error_code())
			else:
				return MoveResponse(1)
		else:
			return MoveResponse(1)

	# Move the gripepr to a given position. Returns error = 0 if no error or error = 1 in case of any kind of error
	# Interface is ROS message
	def callback_move(self, req):

		# Autorelease just in case something was gripped before
		if ControlComm.state.position:
			if autorelease: self.callback_release(MoveRequest(ControlComm.state.position, 50)) # 50 mm/s because this callback can have empty speed
		else:
			rospy.logerr("WSG driver: no prior position information. Check gripper connection")
		return MoveResponse(1)

		self.reset_error()

		if req.speed:
			if ControlComm.send_and_wait_for_fin("MOVE(" + str(req.width) + "," + str(req.speed) + ")"):
				return MoveResponse(ControlComm.state.get_error_code())
			else:
				return MoveResponse(1)
		else:	
			if ControlComm.send_and_wait_for_fin("MOVE(" + str(req.width) + ")"):
				return MoveResponse(ControlComm.state.get_error_code())
			else:
				return MoveResponse(1)

	# Interface is ROS message
	# req.direction can be open or close
	# req.increment is the distance in mm
	# returns ok with error code 0 or 1 if error
	def callback_move_incrementally(self, req):
		self.reset_error()
		if ControlComm.state.position:
			if req.direction == "open":
				return IncrResponse(self.callback_move(MoveRequest(ControlComm.state.position + req.increment,None)).error)
			elif req.direction == "close":
				return IncrResponse(self.callback_move(MoveRequest(ControlComm.state.position - req.increment,None)).error)
		return IncrResponse()

	# Homing the gripper, has to be called at least once after new start of the gripper. Interface is ROS message
	def callback_homing(self, req):
		self.reset_error()
		ControlComm.send_and_wait_for_fin("HOME()")
		return EmptyResponse()

	# Call inmediatelly stop of the gripper. Interface is ROS message
	def callback_stop(self, req):
		if fast_stop: 
			ControlComm.send_without_response("FASTSTOP()") # This one is working better
		else:
			ControlComm.send_without_response("STOP()")
		self.reset_error()
		return EmptyResponse()

	# Set the acceleration as internal driver variable. Interface is ROS message
	def callback_set_acceleration(self, req):
		ControlComm.state.target_acceleration = req.val
		rospy.logerr("WSG Driver: given acceleration ignored, internal acceleration used")
		return ConfResponse(0)

	# Set the force as internal driver variable. Interface is ROS message
	def callback_set_force(self, req):
		ControlComm.state.target_force = req.val
		return ConfResponse(0)

	# Publish drag&bot component status topic to indicate the gripper is in a good shape
	def publish_component_status(self, error = False):
		cm_status = ComponentStatus()
		if error:
			cm_status.status_id = ComponentStatus().ERROR 
		else:
			cm_status.status_id = ComponentStatus().RUNNING
		self.pub_component_status.publish(cm_status)

	# Publish gripper status topic
	def publish_gripper_status(self, state):
		status_msg = Status()
		if state.state: 	status_msg.status = str(state.state)
		if state.position: 	status_msg.width = state.position
		if state.speed:		status_msg.speed = state.speed
		if state.force:		status_msg.force = state.force
		self.pub_gripper_status.publish(status_msg)


def startup(): # Activate auto receiving position, speed, force and gripstate. See gripper documentation for more information
	str_rate = str(rate_hz) # 50 Hz by default
	ControlComm.send_without_response("FSACK()")
	ControlComm.send_without_response("AUTOSEND(\"POS\"," + str_rate + ")")
	ControlComm.send_without_response("AUTOSEND(\"SPEED\"," + str_rate + ")")
	ControlComm.send_without_response("AUTOSEND(\"FORCE\"," + str_rate + ")")
	ControlComm.send_without_response("AUTOSEND(\"GRIPSTATE\"," + str_rate + ")")

if __name__ == '__main__':
	rospy.init_node('wsg50_driver') # Initialise ROS node
	while not ControlComm.connect():
		if rospy.is_shutdown(): sys.exit()
	startup()				# Some initial configuration commands to the gripper
	interface = Interface() # Start ROS interface as independent class
	ControlComm.set_interface(interface)

	rospy.spin() 			# Keep ROS running
