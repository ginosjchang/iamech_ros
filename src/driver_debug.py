#!/usr/bin/env python3
import math
import threading
import signal
import os
import pyads #for plc
import yaml

#ROS 
import rospy
import tf

#ROS msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#msg
from iamech_ros.srv import AGV_Server, AGV_ServerResponse

mutex = threading.Lock()
vel_right_mm_s = 0
vel_left_mm_s = 0
	
# -------------
def cmd_vel_to_wheel_vel(vel_linear_m_s, vel_rotation_rad_s):
	L_mm = rospy.get_param("~wheelbase")
	vel_linear_mm_s = int(vel_linear_m_s * 1000)
	vel_right_mm_s = (2 * vel_linear_mm_s + vel_rotation_rad_s * L_mm) / 2.0
	vel_left_mm_s = (2 * vel_linear_mm_s - vel_rotation_rad_s * L_mm) / 2.0
	return int(vel_right_mm_s), int(vel_left_mm_s)

def cmd_vel_callback(req):
	global vel_right_mm_s, vel_left_mm_s
	vel_right_mm_s, vel_left_mm_s = cmd_vel_to_wheel_vel(vel_linear_m_s = req.linear.x, vel_rotation_rad_s = req.angular.z)

def create_cmd_vel_subscrib():
	rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
	rospy.spin()

def set_AGV_Server(req):
	res = AGV_ServerResponse()
	res.b = req.a
	return res


def create_service():
	service = rospy.Service('ServerOnOff', AGV_Server, set_AGV_Server)
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
	global mutex, vel_left_mm_s, vel_right_mm_s

	L_mm = rospy.get_param("~wheelbase")

	tf_broadcaster = tf.TransformBroadcaster()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

	pos_x = 0
	pos_y = 0
	pos_w = 0

	rate = rospy.Rate(rospy.get_param("~fps"))
	last_time = rospy.Time.now()

	while not rospy.is_shutdown():
		now = rospy.Time.now()

		mutex.acquire()
		vel_right_m_s = vel_right_mm_s / 1000
		vel_left_m_s = vel_left_mm_s / 1000
		mutex.release()

		delta_t = (now - last_time).to_sec()
		last_time = now

		x_dot = (vel_left_m_s + vel_right_m_s) * math.cos(pos_w) / 2
		y_dot = (vel_left_m_s + vel_right_m_s) * math.sin(pos_w) / 2
		w_dot = (1000 / L_mm) * (vel_right_m_s - vel_left_m_s)

		pos_x += x_dot * delta_t
		pos_y += y_dot * delta_t
		pos_w += w_dot * delta_t
		pos_w = pos_w % (math.pi * 2)

		rospy.loginfo("(posx, posy, posw) : (" + str(pos_x) + ", " + str(pos_y) + ", " + str(pos_w) + ")")
		rospy.loginfo("(vel_right, vel_left, vel_w) : (" + str(vel_right_m_s) + ", " + str(vel_left_m_s) + ", " + str(w_dot) + ")")

		quat = create_quat(pos_w)
		tf_broadcaster.sendTransform((pos_x, pos_y, 0), quat, now, "base_link", "odom")

		odom_msg = create_odom_msg(pos_x, pos_y, quat, x_dot, y_dot, w_dot, now)
		
		odom_pub.publish(odom_msg)
		rate.sleep()

def SIGINT_handler(signum, frame):
	rospy.loginfo("Keyboard Interrup")
	rospy.core.signal_shutdown('keyboard interrupt')

def set_param():
	if not rospy.has_param("~yaml_path"): 
		rospy.logerr(".yaml path is needed to setup parameters")
		raise("not yaml path")

	with open(rospy.get_param("~yaml_path"), 'r') as file:
		config = yaml.load(file)
	
	rospy.logdebug("Loading Parameter")
	rospy.set_param("~radius", int(config['AGV']['radius']))
	rospy.logdebug("radius: " + str(config['AGV']['radius']))
	rospy.set_param("~wheelbase", int(config['AGV']['wheelbase']))
	rospy.logdebug("wheelbase: " + str(config['AGV']['wheelbase']))
	rospy.set_param("~fps", float(config['PLC']['fps']))
	rospy.logdebug("fps: " + str(config['PLC']['fps']))
	if not rospy.has_param("~pos_x"): rospy.set_param("~pos_x", 0.0)
	if not rospy.has_param("~pos_y"): rospy.set_param("~pos_y", 0.0)
	if not rospy.has_param("~pos_w"): rospy.set_param("~pos_w", 0.0)
	rospy.set_param("~plc_ip", str(config['PLC']['ip']))
	rospy.logdebug("plc_ip: " + str(config['PLC']['ip']))
	rospy.set_param("~plc_ams_id", str(config['PLC']['ams_id']))
	rospy.logdebug("plc_ams_id: " + str(config['PLC']['ams_id']))
	rospy.set_param("~sender_ams", str(config['PLC']['sender_ams']))
	rospy.logdebug("sender_ams: " + str(config['PLC']['sender_ams']))
	rospy.set_param("~plc_username", str(config['PLC']['username']))
	rospy.logdebug("plc_username: " + str(config['PLC']['username']))
	rospy.set_param("~plc_password", str(config['PLC']['password']))
	rospy.logdebug("plc_password: " + str(config['PLC']['password']))
	rospy.set_param("~plc_route_name", str(config['PLC']['route_name']))
	rospy.logdebug("plc_route_name: " + str(config['PLC']['route_name']))
	rospy.set_param("~plc_hostname", str(config['PLC']['hostname']))
	rospy.logdebug("plc_hostname: " + str(config['PLC']['hostname']))

if __name__ == '__main__':

	rospy.init_node('iAmech_Debug', log_level=rospy.DEBUG)

	threads = []

	set_param()
	# connectToPLC()

	isTest = rospy.get_param("~isTest")
	threads.append(threading.Thread(target=create_cmd_vel_subscrib))
	threads.append(threading.Thread(target=create_tf_odom_publisher))
	# threads.append(threading.Thread(target=print_agv_status))
	threads.append(threading.Thread(target=create_service))

	signal.signal(signal.SIGINT, SIGINT_handler)

	for thread in threads:
		thread.start()

	for thread in threads:
		thread.join()
		print("Kill thread")
	
	print("end of node")