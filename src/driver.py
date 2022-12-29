#!/usr/bin/env python3
import math
import threading
import signal
import os
import pyads #for plc

import yamlreader
import connect
import odometry

#ROS 
import rospy
import tf

#ROS msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#User defined
from iamech_ros.srv import AGV_Server, AGV_ServerResponse
from iamech_ros.msg import PLCStatus

### ------Velocity------
## Convert cmd_vel to two wheel machine velocity.
def cmd_vel_to_wheel_vel(vel_linear_m_s, vel_rotation_rad_s):
	temp = 2 * vel_linear_m_s * 1000
	temp2 = vel_rotation_rad_s * rospy.get_param("~wheelbase")
	vel_right_mm_s = (temp + temp2) / 2
	vel_left_mm_s = (temp - temp2) / 2
	return int(vel_right_mm_s), int(vel_left_mm_s)

## Set velocity in plc
def amm_drive(vel_left_mm_s, vel_right_mm_s):
	global plc, mutex
	connect.set_parameter([".SLAM_L[2]", ".SLAM_R[2]"], [vel_left_mm_s, vel_right_mm_s])

## Callback function
def cmd_vel_callback(req):
	vel_right_mm_s, vel_left_mm_s = cmd_vel_to_wheel_vel(vel_linear_m_s = req.linear.x, vel_rotation_rad_s = req.angular.z)
	amm_drive(vel_left_mm_s, vel_right_mm_s)

## Subscrib
def create_cmd_vel_subscrib():
	rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
	rospy.spin()

### ------Odometry------
def odom_thread():
	L_mm = rospy.get_param("~wheelbase")

	tf_broadcaster = tf.TransformBroadcaster()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	plc_pub = rospy.Publisher("plc_status", PLCStatus, queue_size=50)

	rate = rospy.Rate(rospy.get_param("~fps"))
	last_plc = connect.get_status()

	while not rospy.is_shutdown():
		now = rospy.Time.now()

		current_plc = connect.get_status()

		diff_right = current_plc.right.pos - last_plc.right.pos
		diff_left = current_plc.left.pos - last_plc.left.pos

		position = odometry.pos_update(diff_right, diff_left, L_mm)

		vel_x = (current_plc.left.velocity + current_plc.right.velocity) * math.cos(position[2]) / 2000.0
		vel_y = (current_plc.left.velocity + current_plc.right.velocity) * math.sin(position[2]) / 2000.0
		vel_w = 1.0 / L_mm * (current_plc.right.velocity - current_plc.left.velocity)
		quat = odometry.create_quat(position[2])
		tf_broadcaster.sendTransform((position[0], position[1], 0), quat, now, "base_link", "odom")
		odom_msg = odometry.create_odom_msg(position[0], position[1], quat, vel_x, vel_y, vel_w, now)

		last_plc = current_plc

		plc_pub.publish(current_plc)
		odom_pub.publish(odom_msg)
		rate.sleep()

### ------Service------

def set_serveOn(req):
	result = AGV_ServerResponse()
	result.b = connect.set_parameter(".bSLAM_ServeON", req.a)
	return result

def create_service():
	service = rospy.Service('~set_ServeON', AGV_Server, set_serveOn)
	odometry.service_on()
	rospy.spin()

def SIGINT_handler(signum, frame):
	rospy.loginfo("Keyboard Interrup")
	rospy.core.signal_shutdown('keyboard interrupt')

if __name__ == '__main__':

	rospy.init_node('iAmech', log_level=rospy.DEBUG)

	threads = []

	yamlreader.yaml_read()
	connect.connectToPLC()
	# connect.connectVirtual()

	threads.append(threading.Thread(target=create_cmd_vel_subscrib))
	threads.append(threading.Thread(target=odom_thread))
	threads.append(threading.Thread(target=create_service))

	signal.signal(signal.SIGINT, SIGINT_handler)

	for thread in threads:
		thread.start()

	for thread in threads:
		thread.join()
		print("Kill thread")

	connect.close()
	print("end of node")
