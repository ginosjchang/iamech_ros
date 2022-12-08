#!/usr/bin/env python3

from ctypes.wintypes import INT
from lib2to3.pgen2 import driver
import math,time
from math import sin, cos, pi
from tkinter import SEL
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import pyads
from std_msgs.msg import Int8
import os
from time import sleep
import threading
import signal

L = 630 #mm
R = 85 #mm

mutex = threading.Lock()

vel_x = 0
vel_y = 0
pos_x = 0
pos_y = 0
pos_theta = 0

threads = []

#Unit of linear_vel is mm/s. Unit of angular_vel is rad/s
def cvt_cmd_to_wheel_vel(linear_vel: int, angular_vel):
	vr = 20*(2 * linear_vel + angular_vel * L) / R 
	vl = 20*(2 * linear_vel - angular_vel * L) / R 
	return int(vr), int(vl)

#Unit of angle is rad/s
def create_quat(angle):
	return tf.transformations.quaternion_from_euler(0, 0, angle)

def sendTf(pos_x, pos_y, quat, broadcaster):
	broadcaster.sendTransform((pos_x, pos_y, 0), quat, rospy.Time.now(), "base_link", "odom")

def create_odom_msg(pos_x, pos_y, quat, vel_x, vel_y, angular):
	odom = Odometry()
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "odom"

    # set the position
	odom.pose.pose = Pose(Point(pos_x, pos_y, 0.), Quaternion(*quat))

    # set the velocity
	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(vel_x, vel_y, 0), Vector3(0, 0, angular))

	return odom

def keep_publish_tf_thread():
	odom_broadcaster = tf.TransformBroadcaster()
	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		mutex.acquire()
		quat = create_quat(pos_theta)
		sendTf(pos_x, pos_y, quat, odom_broadcaster)
		mutex.release()
		rate.sleep()

def keep_publish_odom_thread():
	global pos_x, pos_y, vel_x, vel_y
	odom_publisher = rospy.Publisher("odom", Odometry, queue_size=50)
	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		odom_quat = create_quat(pos_theta)
		odom_msg = create_odom_msg(pos_x, pos_y, odom_quat, vel_x, vel_y, 0)
		odom_publisher.publish(odom_msg)
		rate.sleep()
	
def keep_forward_thread():
	rate = rospy.Rate(1.0)
	global pos_x, pos_y, vel_x, vel_y
	while not rospy.is_shutdown():
		mutex.acquire()
		pos_x += vel_x
		pos_y += vel_y
		mutex.release()
		rate.sleep()

def SIGINT_handler(signum, frame):
	print("Kill threads")
	for thread in threads:
		thread.terminate()

class iAmechROS():
	def __init__(self):
		rospy.init_node('iAmech', log_level=rospy.DEBUG)

		# Connection
		self.PLC_AMS_ID = '192.168.100.100.1.1'
		self.PLC_IP = '192.168.100.100'
		self.connectToPLC()

		# Subscribe to topics	
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

		# Cleanup when terminating the node
		rospy.on_shutdown(self.shutdown)

		self.plc.write_by_name(".bSLAM_ServeON", 1, pyads.PLCTYPE_BOOL)

		while not rospy.is_shutdown():
			self.poll()
			sleep(0.5)
				
		rospy.spin()
	
	def connectToPLC(self):
		pyads.open_port()
		pyads.set_local_address('1.2.3.4.1.1')
		pyads.close_port()

		pyads.open_port()
		pyads.add_route(self.PLC_AMS_ID, self.PLC_IP)
		pyads.close_port()

		SENDER_AMS = '1.2.3.4.1.1'
		PLC_USERNAME = 'Administrator'
		PLC_PASSWORD = '1'
		ROUTE_NAME = 'RouteToMyPC'
		HOSTNAME = '192.168.100.191'

		pyads.add_route_to_plc(SENDER_AMS, HOSTNAME, self.PLC_IP, PLC_USERNAME, PLC_PASSWORD, route_name=ROUTE_NAME)

		print("Initialize connection to PLC")
		self.plc = pyads.Connection(self.PLC_AMS_ID, 801, self.PLC_IP)
		self.plc.open()
		print("Connection built!")
	
	def poll(self):
		os.system('clear')
		print("Left wheel:")
		print("\tPos: ", self.plc.read_by_name(".SLAM_L[14]", pyads.PLCTYPE_DINT))
		print("\tVelocity", self.plc.read_by_name(".SLAM_L[15]", pyads.PLCTYPE_DINT))
		print("\tvoltage: ", self.plc.read_by_name(".SLAM_L[18]", pyads.PLCTYPE_DINT))
		print("Right wheel:")
		print("\tPos: ", self.plc.read_by_name(".SLAM_R[14]", pyads.PLCTYPE_DINT))
		print("\tVelocity", self.plc.read_by_name(".SLAM_R[15]", pyads.PLCTYPE_DINT))
		print("\tvoltage: ", self.plc.read_by_name(".SLAM_R[18]", pyads.PLCTYPE_DINT))
		# l_volt = self.plc.read_by_name(".SLAM_L[18]", pyads.PLCTYPE_DINT)
		# r_volt = self.plc.read_by_name(".SLAM_R[18]", pyads.PLCTYPE_DINT)
		# print("Current voltage is {}v {}v".format(l_volt, r_volt))

	def cmdVelCallback(self, req):
		self.last_cmd_vel = rospy.Time.now()
		
		L = 630 # mm
		R = 85 # mm
		
		x = req.linear.x	# m/s
		th = req.angular.z # rad/s

		v = int(x * 1000) # mm/s
		
		vr = 20*(2*v + th*L) / (R) 
		vl = 20*(2*v - th*L) / (R) 
		# vr = (2*v + th*L) / 2
		# vl = (2*v - th*L) / 2 
		# vr = 40*v/R
		# vl = 40*v/R
		
		left_wheel = int(vl)
		right_wheel = int(vr)
		self.plc.write_by_name(".SLAM_L[2]", left_wheel, pyads.PLCTYPE_DINT)
		self.plc.write_by_name(".SLAM_R[2]", right_wheel, pyads.PLCTYPE_DINT)
	
	def shutdown(self):
		try:
			rospy.loginfo("Stopping robot...")
			self.plc.write_by_name(".SLAM_L[2]", 0, pyads.PLCTYPE_DINT)
			self.plc.write_by_name(".SLAM_R[2]", 0, pyads.PLCTYPE_DINT)
			self.plc.write_by_name(".bSLAM_ServeON", 0, pyads.PLCTYPE_BOOL)
			self.plc.close()
			self.cmd_vel_pub.Publish(Twist())
			rospy.sleep(2)
		except:
			pass
		rospy.loginfo("Shutting down iAmech node...")


if __name__ == '__main__':
	rospy.init_node('iAmech', log_level=rospy.DEBUG)
	threads.append(threading.Thread(target=keep_publish_tf_thread))
	threads.append(threading.Thread(target=keep_publish_odom_thread))
	threads.append(threading.Thread(target=keep_forward_thread))

	signal.signal(signal.SIGINT, SIGINT_handler)

	for thread in threads:
		thread.start()
	
	for thread in threads:
		thread.join()