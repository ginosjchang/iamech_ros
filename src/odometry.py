import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import math
from threading import Lock

from iamech_ros.srv import Position, PositionResponse

position = [0.0, 0.0, 0.0] # x, y, w
odom_mutex = Lock()

def create_quat(rad):
	return tf.transformations.quaternion_from_euler(0, 0, rad * math.pi / 180.0)

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

def pos_update(diff_right, diff_left, L_mm):
    global position, odom_mutex
    odom_mutex.acquire()
    if diff_right == diff_left:
        dx_mm = diff_right * math.cos(position[2])
        dy_mm = diff_right * math.sin(position[2])
        dw_rad = 0
    elif diff_right + diff_left == 0:
        dx_mm = 0
        dy_mm = 0
        dw_rad = 2.0 * diff_right / L_mm
    else:
        temp = L_mm / 2.0 * abs((diff_left + diff_right) / (diff_left - diff_right))
        theta = (diff_right - diff_left) / float(L_mm)
        dw_rad = theta
        if abs(diff_left) < abs(diff_right): #ture left
            theta *= -1
            dx_ = temp * math.sin(theta)
            dy_ = temp * (1 - math.cos(theta))
        else: #Turn right
            dx_ = temp * math.sin(theta)
            dy_ = temp * (math.cos(theta) - 1)
        
        dx_mm = dx_ * math.cos(position[2]) - dy_ * math.sin(position[2])
        dy_mm = dx_ * math.sin(position[2]) + dy_ * math.cos(position[2])
    position[0] += dx_mm / 1000
    position[1] += dy_mm / 1000
    position[2] += dw_rad
    position[2] = position[2] % (math.pi * 2)

    result = position.copy()
    odom_mutex.release()
    return result

def set_position(x, y, w):
    global position, odom_mutex
    odom_mutex.acquire()
    position[0] = x
    position[1] = y
    position[2] = w
    odom_mutex.release()

def get_position():
    global odom_mutex, position
    odom_mutex.acquire()
    p_x = position[0]
    p_y = position[1]
    p_w = position[2]
    odom_mutex.release()
    return p_x, p_y, p_w

def get_position_service(req):
    p = get_position()
    res = PositionResponse(p[0], p[1], p[2])
    return res

def set_position_service(req):
    set_position(req.x, req.y, req.w)
    p = get_position()
    res = PositionResponse(p[0], p[1], p[2])
    return res
    
def service_on():
    rospy.Service('~get_position', Position, get_position_service)
    rospy.Service('~set_position', Position, set_position_service)