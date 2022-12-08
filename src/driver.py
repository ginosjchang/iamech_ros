#!/usr/bin/env python3

from ctypes.wintypes import INT
import math,time
from math import sin, cos, pi
from tkinter import SEL
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import pyads
from std_msgs.msg import Int8
import os

import threading
import signal

import rospy

L = 630 #mm
R = 85 #mm

mutex = threading.Lock()

vel_right = 0
vel_left = 0

vel_x = 0
vel_y = 0
pos_x = 0
pos_y = 0
pos_w = 0

# -------------
def cmd_vel_to_wheel_vel(vel_linear_m_s, vel_rotation_rad_s):
	vel_linear_mm_s = int(vel_linear_m_s * 1000)
	vel_right = 20 * (2 * vel_linear_mm_s + vel_rotation_rad_s * L) / R
	vel_left = 20 * (2 * vel_linear_mm_s - vel_rotation_rad_s * L) / R
	return int(vel_right), int(vel_left)

def cmd_vel_callback(req):
	global vel_right, vel_left
	mutex.acquire()
	vel_right, vel_left = cmd_vel_to_wheel_vel(vel_linear_m_s = req.linear.x, vel_rotation_rad_s = req.angular.z)
	print("set wheel speed: ", vel_right, " , ", vel_left)
	mutex.release()

def create_cmd_vel_subscrib():
	rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
	rospy.spin()
# --------------

#Unit of angle is rad/s
def create_quat(pos_w):
	return tf.transformations.quaternion_from_euler(0, 0, pos_w)

def create_odom_msg(pos_x, pos_y, quat, vel_x, vel_y, angular, now):
	odom = Odometry()
	odom.header.stamp = now
	odom.header.frame_id = "odom"

    # set the position
	odom.pose.pose = Pose(Point(pos_x, pos_y, 0.), Quaternion(*quat))

    # set the velocity
	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(vel_x, vel_y, 0), Vector3(0, 0, angular))

	return odom

def create_tf_odom_publisher():
	global vel_right, vel_left

	tf_broadcaster = tf.TransformBroadcaster()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

	pos_x = 0
	pos_y = 0
	pos_w = 0

	R = 0.085
	L = 0.6

	rate = rospy.Rate(1.0)
	last_time = rospy.Time.now()

	while not rospy.is_shutdown():
		now = rospy.Time.now()

		mutex.acquire()
		v_r = vel_right / 1000
		v_l = vel_left / 1000
		mutex.release()

		delta_t = (now - last_time).to_sec()
		last_time = now

		x_dot = (v_l + v_r) * math.cos(pos_w) / 2
		y_dot = (v_l + v_r) * math.sin(pos_w) / 2
		w_dot = (1 / L) * (v_r - v_l)

		pos_x += x_dot * delta_t
		pos_y += y_dot * delta_t
		pos_w += w_dot * delta_t
		pos_w = pos_w % (math.pi * 2)

		quat = create_quat(pos_w)
		tf_broadcaster.sendTransform((pos_x, pos_y, 0), quat, now, "base_link", "odom")

		odom_msg = create_odom_msg(pos_x, pos_y, quat, x_dot, y_dot, w_dot, now)
		
		odom_pub.publish(odom_msg)
		rate.sleep()

def SIGINT_handler(signum, frame):
	print('\nkeyboard interrup')
	rospy.core.signal_shutdown('keyboard interrupt')

if __name__ == '__main__':
	rospy.init_node('iAmech', log_level=rospy.DEBUG)

	threads = []

	threads.append(threading.Thread(target=create_cmd_vel_subscrib))
	threads.append(threading.Thread(target=create_tf_odom_publisher))

	signal.signal(signal.SIGINT, SIGINT_handler)

	for thread in threads:
		thread.start()

	for thread in threads:
		thread.join()
		print("Kill thread")
	print("end of node")