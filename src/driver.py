#!/usr/bin/env python3
import math
import threading
import signal
import os
import pyads #for plc

#ROS 
import rospy
import tf

#ROS msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

mutex = threading.Lock()

isTest = None

vel_right_mm_s = 0
vel_left_mm_s = 0

plc = None
	
# -------------
def cmd_vel_to_wheel_vel(vel_linear_m_s, vel_rotation_rad_s):
	L_mm = rospy.get_param("~length")
	R_mm = rospy.get_param("~radius")
	vel_linear_mm_s = int(vel_linear_m_s * 1000)
	vel_right_mm_s = 20 * (2 * vel_linear_mm_s + vel_rotation_rad_s * L_mm) / R_mm
	vel_left_mm_s = 20 * (2 * vel_linear_mm_s - vel_rotation_rad_s * L_mm) / R_mm
	return int(vel_right_mm_s), int(vel_left_mm_s)

def AGV_drive(vel_left_mm_s, vel_right_mm_s):
	global plc
	plc.write_by_name(".SLAM_L[2]", vel_left_mm_s, pyads.PLCTYPE_DINT)
	plc.write_by_name(".SLAM_R[2]", vel_right_mm_s, pyads.PLCTYPE_DINT)

def cmd_vel_callback(req):
	global vel_right_mm_s, vel_left_mm_s, isTest
	mutex.acquire()
	vel_right_mm_s, vel_left_mm_s = cmd_vel_to_wheel_vel(vel_linear_m_s = req.linear.x, vel_rotation_rad_s = req.angular.z)
	if not isTest:
		AGV_drive(vel_left_mm_s, vel_right_mm_s)
	# print("set wheel speed: ", vel_right_mm_s, " , ", vel_left_mm_s)
	mutex.release()

def create_cmd_vel_subscrib():
	rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
	rospy.spin()
# --------------

def create_quat(degree):
	return tf.transformations.quaternion_from_euler(0, 0, degree)

def create_odom_msg(pos_x, pos_y, quat, vel_x, vel_y, vel_w, now):
	odom = Odometry()
	odom.header.stamp = now
	odom.header.frame_id = "odom"

    # set the position
	odom.pose.pose = Pose(Point(pos_x, pos_y, 0.), Quaternion(*quat))

    # set the velocity
	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(vel_x, vel_y, 0), Vector3(0, 0, vel_w))

	return odom

def create_tf_odom_publisher():
	global vel_right_mm_s, vel_left_mm_s, plc, isTest

	if not isTest:
		vel_left_mm_s = plc.read_by_name(".SLAM_L[15]", pyads.PLCTYPE_DINT)
		vel_right_mm_s = plc.read_by_name(".SLAM_R[15]", pyads.PLCTYPE_DINT)


	L_mm = rospy.get_param("~length")

	tf_broadcaster = tf.TransformBroadcaster()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

	pos_x = rospy.get_param("~pos_x")
	pos_y = rospy.get_param("~pos_y")
	pos_w = rospy.get_param("~pos_w")

	rate = rospy.Rate(rospy.get_param("~fps"))
	last_time = rospy.Time.now()

	while not rospy.is_shutdown():
		now = rospy.Time.now()

		mutex.acquire()
		v_r = vel_right_mm_s / 1000
		v_l = vel_left_mm_s / 1000
		mutex.release()

		delta_t = (now - last_time).to_sec()
		last_time = now

		x_dot = (v_l + v_r) * math.cos(pos_w) / 2
		y_dot = (v_l + v_r) * math.sin(pos_w) / 2
		w_dot = (1000 / L_mm) * (v_r - v_l)

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

def connectToPLC():
	global plc

	PLC_AMS_ID = rospy.get_param("~plc_ams_id")
	PLC_IP = rospy.get_param("~plc_ip")

	SENDER_AMS = rospy.get_param("~sender_ams")
	PLC_USERNAME = rospy.get_param("~plc_username")
	PLC_PASSWORD = rospy.get_param("~plc_password")
	ROUTE_NAME = rospy.get_param("~plc_route_name")
	HOSTNAME = rospy.get_param("~plc_hostname")

	pyads.open_port()
	pyads.set_local_address(SENDER_AMS)
	pyads.close_port()

	pyads.open_port()
	pyads.add_route(PLC_AMS_ID, PLC_IP)
	pyads.close_port()

	

	pyads.add_route_to_plc(SENDER_AMS, HOSTNAME, PLC_IP, PLC_USERNAME, PLC_PASSWORD, route_name=ROUTE_NAME)

	print("Initialize connection to PLC")
	plc = pyads.Connection(PLC_AMS_ID, 801, PLC_IP)
	plc.open()
	print("Connection built!")

def set_param():
	print("AGV Parameters: ")
	if not rospy.has_param("~radius"): rospy.set_param("~radius", 85)
	print("\tRadius: ", rospy.get_param("~radius"))

	if not rospy.has_param("~length"): rospy.set_param("~length", 630)
	print("\tLength: ", rospy.get_param("~length"))

	if not rospy.has_param("~fps"): rospy.set_param("~fps", 1.0)
	print("\tPublish fps: ", rospy.get_param("~fps"))

	if not rospy.has_param("~pos_x"): rospy.set_param("~pos_x", 0.0)
	print("\tpos_x: ", rospy.get_param("~pos_x"))

	if not rospy.has_param("~pos_y"): rospy.set_param("~pos_y", 0.0)
	print("\tpos_y: ", rospy.get_param("~pos_y"))
	
	if not rospy.has_param("~pos_w"): rospy.set_param("~pos_w", 0.0)
	print("\tpow_w: ", rospy.get_param("~pos_w"))

	print("PLC Parameters: ")
	if not rospy.has_param("~plc_ip"): rospy.set_param("~plc_ip", "192.168.100.100")
	print("\tPLC IP: ", rospy.get_param("~plc_ip"))

	if not rospy.has_param("~plc_ams_id"): rospy.set_param("~plc_ams_id", "192.168.100.100.1.1")
	print("\tPLC AMS ID: ", rospy.get_param("~plc_ams_id"))

	if not rospy.has_param("~sender_ams"): rospy.set_param("~sender_ams", "1.2.3.4.1.1")
	print("\tSENDER AMS: ", rospy.get_param("~sender_ams"))

	if not rospy.has_param("~plc_username"): rospy.set_param("~plc_username", "Administrator")
	print("\tPLC_USERNAME: ", rospy.get_param("~plc_username"))

	if not rospy.has_param("~plc_password"): rospy.set_param("~plc_password", "1")
	print("\tPLC_PASSWORD: ", rospy.get_param("~plc_password"))

	if not rospy.has_param("~plc_route_name"): rospy.set_param("~plc_route_name", "RouteToMyPC")
	print("\tROUTE_NAME: ", rospy.get_param("~plc_route_name"))

	if not rospy.has_param("~plc_hostname"): rospy.set_param("~plc_hostname", "192.168.100.191")
	print("\tHOSTNAME: ", rospy.get_param("~plc_hostname"))	

if __name__ == '__main__':

	rospy.init_node('iAmech', log_level=rospy.DEBUG)

	threads = []

	threads.append(threading.Thread(target=create_cmd_vel_subscrib))
	threads.append(threading.Thread(target=create_tf_odom_publisher))

	set_param()

	if not rospy.has_param("~isTest"): rospy.set_param("~isTest", True)
	isTest = rospy.get_param("~isTest")

	if not isTest: connectToPLC()

	signal.signal(signal.SIGINT, SIGINT_handler)

	for thread in threads:
		thread.start()

	for thread in threads:
		thread.join()
		print("Kill thread")
	print("end of node")