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

#User defined
from iamech_ros.srv import AGV_Server, AGV_ServerResponse
from iamech_ros.msg import PLCState

mutex = threading.Lock()
plc = None
	
# -------------
def cmd_vel_to_wheel_vel(vel_linear_m_s, vel_rotation_rad_s):
	L_mm = rospy.get_param("~wheelbase")
	R_mm = rospy.get_param("~radius")
	vel_linear_mm_s = int(vel_linear_m_s * 1000)
	vel_right_mm_s = (2 * vel_linear_mm_s + vel_rotation_rad_s * L_mm) / 2
	vel_left_mm_s = (2 * vel_linear_mm_s - vel_rotation_rad_s * L_mm) / 2
	return int(vel_right_mm_s), int(vel_left_mm_s)

def AGV_drive(vel_left_mm_s, vel_right_mm_s):
	global plc, mutex
	mutex.acquire()
	plc.write_by_name(".SLAM_L[2]", vel_left_mm_s, pyads.PLCTYPE_DINT)
	plc.write_by_name(".SLAM_R[2]", vel_right_mm_s, pyads.PLCTYPE_DINT)
	mutex.release()

def cmd_vel_callback(req):
	vel_right_mm_s, vel_left_mm_s = cmd_vel_to_wheel_vel(vel_linear_m_s = req.linear.x, vel_rotation_rad_s = req.angular.z)
	AGV_drive(vel_left_mm_s, vel_right_mm_s)

def create_cmd_vel_subscrib():
	rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
	rospy.spin()

def set_AGV_Server(req):
	return AGV_ServerResponse(req.a)


def create_service():
	service = rospy.Service('ServerOnOff', AGV_Server, set_AGV_Server)
	rospy.spin()

# --------------
def get_PLC_state():
	global mutex
	mutex.acquire()
	state = PLCState()
	state.right.bReady = plc.read_by_name(".SLAM_R[11]", pyads.PLCTYPE_DINT)
	state.right.bMoving = plc.read_by_name(".SLAM_R[12]", pyads.PLCTYPE_DINT)
	state.right.bError = plc.read_by_name(".SLAM_R[13]", pyads.PLCTYPE_DINT)
	state.right.pos = plc.read_by_name(".SLAM_R[14]", pyads.PLCTYPE_DINT)
	state.right.velocity = plc.read_by_name(".SLAM_R[15]", pyads.PLCTYPE_DINT)
	state.right.ErrorCode = plc.read_by_name(".SLAM_R[16]", pyads.PLCTYPE_DINT)
	state.right.temperature = plc.read_by_name(".SLAM_R[17]", pyads.PLCTYPE_DINT)
	state.right.volt = plc.read_by_name(".SLAM_R[18]", pyads.PLCTYPE_DINT)

	state.left.bReady = plc.read_by_name(".SLAM_L[11]", pyads.PLCTYPE_DINT)
	state.left.bMoving = plc.read_by_name(".SLAM_L[12]", pyads.PLCTYPE_DINT)
	state.left.bError = plc.read_by_name(".SLAM_L[13]", pyads.PLCTYPE_DINT)
	state.left.pos = plc.read_by_name(".SLAM_L[14]", pyads.PLCTYPE_DINT)
	state.left.velocity = plc.read_by_name(".SLAM_L[15]", pyads.PLCTYPE_DINT)
	state.left.ErrorCode = plc.read_by_name(".SLAM_L[16]", pyads.PLCTYPE_DINT)
	state.left.temperature = plc.read_by_name(".SLAM_L[17]", pyads.PLCTYPE_DINT)
	state.left.volt = plc.read_by_name(".SLAM_L[18]", pyads.PLCTYPE_DINT)
	mutex.release()
	return state

def print_PLC_state(state):
	print("Right:")
	print("\tbReady: ", state.right.bReady)
	print("\tbMoving: ", state.right.bMoving)
	print("\tbError: ", state.right.bError)
	print("\tpos: ", state.right.pos)
	print("\tvelocity: ", state.right.velocity)
	print("\tErrorCode: ", state.right.ErrorCode)
	print("\ttemperature: ", state.right.temperature)
	print("\tvolt: ", state.right.volt)
	print("Left")
	print("\tbReady: ", state.left.bReady)
	print("\tbMoving: ", state.left.bMoving)
	print("\tbError: ", state.left.bError)
	print("\tpos: ", state.left.pos)
	print("\tvelocity: ", state.left.velocity)
	print("\tErrorCode: ", state.left.ErrorCode)
	print("\ttemperature: ", state.left.temperature)
	print("\tvolt: ", state.left.volt)

def create_PLC_state_subscrib():
	rospy.Subscriber("plc_state", PLCState, print_PLC_state)
	rospy.spin()

def create_quat(rad):
	return tf.transformations.quaternion_from_euler(0, 0, rad)

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
	global plc, mutex

	L_mm = rospy.get_param("~wheelbase")

	tf_broadcaster = tf.TransformBroadcaster()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	plc_pub = rospy.Publisher("plc_state", PLCState, queue_size=50)

	pos_x = 0
	pos_y = 0
	pos_w = 0

	rate = rospy.Rate(rospy.get_param("~fps"))
	last_plc = get_PLC_state()

	while not rospy.is_shutdown():
		now = rospy.Time.now()

		current_plc = get_PLC_state()

		vel_x = (current_plc.left.velocity + current_plc.right.velocity) * math.cos(pos_w) / 2000.0
		vel_y = (current_plc.left.velocity + current_plc.right.velocity) * math.sin(pos_w) / 2000.0
		vel_w = 1.0 / L_mm * (current_plc.right.velocity - current_plc.left.velocity)

		diff_right = current_plc.right.pos - last_plc.right.pos
		diff_left = current_plc.left.pos - last_plc.left.pos

		if diff_right == diff_left:
			dx_mm = diff_right * math.cos(pos_w)
			dy_mm = diff_right * math.sin(pos_w)
			dw_rad = 0
		elif diff_right + diff_left == 0:
			dx_mm = 0
			dy_mm = 0
			dw_rad = 2.0 * diff_right / L_mm
		else:
			temp = L_mm / 2.0 * abs((diff_left + diff_right) / (diff_left - diff_right))
			theta = (diff_right - diff_left) / float(L_mm) #ture left is positive.Otherwise, negative
			dw_rad = theta
			if abs(diff_left) < abs(diff_right): #ture left
				dx_ = temp * math.sin(theta)
				dy_ = temp * (1 - math.cos(theta))
			else: #Turn right
				theta *= -1
				dx_ = temp * math.sin(theta)
				dy_ = temp * (math.cos(theta) - 1)
			
			dx_mm = dx_ * math.cos(pos_w) - dy_ * math.sin(pos_w)
			dy_mm = dx_ * math.sin(pos_w) + dy_ * math.cos(pos_w)

		pos_x += dx_mm / 1000
		pos_y += dy_mm / 1000
		pos_w += dw_rad
		pos_w = pos_w % (math.pi * 2)

		quat = create_quat(pos_w)
		tf_broadcaster.sendTransform((pos_x, pos_y, 0), quat, now, "base_link", "odom")
		odom_msg = create_odom_msg(pos_x, pos_y, quat, vel_x, vel_y, vel_w, now)

		last_plc = current_plc

		plc_pub.publish(current_plc)
		odom_pub.publish(odom_msg)
		rate.sleep()

def SIGINT_handler(signum, frame):
	rospy.loginfo("Keyboard Interrup")
	rospy.core.signal_shutdown('keyboard interrupt')

def connectToPLC():
	global plc
	rospy.loginfo("Initialize connection to backoff PLC")

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

	plc = pyads.Connection(PLC_AMS_ID, 801, PLC_IP)
	plc.open()
	rospy.loginfo("Connection built!")

	plc.write_by_name(".bSLAM_ServeON", 1, pyads.PLCTYPE_BOOL)

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

	rospy.init_node('iAmech', log_level=rospy.DEBUG)

	threads = []

	set_param()
	connectToPLC()

	isTest = rospy.get_param("~isTest")
	threads.append(threading.Thread(target=create_cmd_vel_subscrib))
	threads.append(threading.Thread(target=create_tf_odom_publisher))
	threads.append(threading.Thread(target=create_PLC_state_subscrib))

	signal.signal(signal.SIGINT, SIGINT_handler)

	for thread in threads:
		thread.start()

	for thread in threads:
		thread.join()
		print("Kill thread")
	
	plc.write_by_name(".bSLAM_ServeON", 0, pyads.PLCTYPE_BOOL)
	plc.close()
	print("end of node")
