import pyads
import rospy
from threading import Lock

from iamech_ros.msg import PLCStatus

plc = None
plc_mutex = Lock()
plc_data_tpye = {
	".bSLAM_ServeON": pyads.PLCTYPE_BOOL,
	".SLAM_R[2]": pyads.PLCTYPE_DINT,
	".SLAM_R[11]": pyads.PLCTYPE_DINT,
	".SLAM_R[12]": pyads.PLCTYPE_DINT,
	".SLAM_R[13]": pyads.PLCTYPE_DINT,
	".SLAM_R[14]": pyads.PLCTYPE_DINT,
	".SLAM_R[15]": pyads.PLCTYPE_DINT,
	".SLAM_R[16]": pyads.PLCTYPE_DINT,
	".SLAM_R[17]": pyads.PLCTYPE_DINT,
	".SLAM_R[18]": pyads.PLCTYPE_DINT,
	".SLAM_L[2]": pyads.PLCTYPE_DINT,
	".SLAM_L[11]": pyads.PLCTYPE_DINT,
	".SLAM_L[12]": pyads.PLCTYPE_DINT,
	".SLAM_L[13]": pyads.PLCTYPE_DINT,
	".SLAM_L[14]": pyads.PLCTYPE_DINT,
	".SLAM_L[15]": pyads.PLCTYPE_DINT,
	".SLAM_L[16]": pyads.PLCTYPE_DINT,
	".SLAM_L[17]": pyads.PLCTYPE_DINT,
	".SLAM_L[18]": pyads.PLCTYPE_DINT
}

class VirtualPLC:
	def __init__(self):
		self.memory = {
			".bSLAM_ServeON": 0,
			".SLAM_R[11]": 0,
			".SLAM_R[12]": 0,
			".SLAM_R[13]": 0,
			".SLAM_R[14]": 0,
			".SLAM_R[15]": 0,
			".SLAM_R[16]": 0,
			".SLAM_R[17]": 0,
			".SLAM_R[18]": 0,
			".SLAM_L[11]": 0,
			".SLAM_L[12]": 0,
			".SLAM_L[13]": 0,
			".SLAM_L[14]": 0,
			".SLAM_L[15]": 0,
			".SLAM_L[16]": 0,
			".SLAM_L[17]": 0,
			".SLAM_L[18]": 0
		}

	def write_by_name(self, name, value, dtype):
		if name.__eq__(".SLAM_L[2]"):
			self.memory[".SLAM_L[15]"] = value
		elif name.__eq__(".SLAM_R[2]"):
			self.memory[".SLAM_R[15]"] = value
		else:
			self.memory[name] = value

		# print("write " + str(name) + "  " + str(value))
	
	def read_by_name(self, name, dtype):
		if name == ".SLAM_L[2]":
			return self.memory[".SLAM_L[15]"]
		elif name == ".SLAM_R[2]":
			return self.memory[".SLAM_R[15]"]
		else:
			return self.memory[name]
	
	def close(self):
		pass

#Coneect with plc and busy wait until connect success.
def connectToPLC():
	global plc
	rospy.loginfo("Initialize connection to beckhoff PLC")

	#Get PLC parameters from parameter server
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

	rospy.loginfo("Connecting.......")
	plc = pyads.Connection(PLC_AMS_ID, 801, PLC_IP)
	plc.open()
	rospy.loginfo("Connection built!")

	#Set the amm serverOn is True
	plc.write_by_name(".bSLAM_ServeON", 1, pyads.PLCTYPE_BOOL)

def connectVirtual():
	global plc
	plc = VirtualPLC()
	rospy.loginfo("Connect with Virtual PLC")

def close():
	global plc_mutex, plc
	plc_mutex.acquire()
	if not plc:
		plc_mutex.release()
		return
	plc.write_by_name(".bSLAM_ServerON", 0, pyads.PLCTYPE_BOOL)
	plc.close()
	plc_mutex.release()

def set_parameter(name, value):
	global plc_mutex, plc, plc_data_tpye
	plc_mutex.acquire()
	if not plc:
		plc_mutex.release()
		return None
	if type(name) == type([]):
		result = []
		for i in range(len(name)):
			plc.write_by_name(name[i], value[i], plc_data_tpye[name[i]])
			result.append(plc.read_by_name(name[i], plc_data_tpye[name[i]]))
	else:
		plc.write_by_name(name, value, plc_data_tpye[name])
		result = plc.read_by_name(name, plc_data_tpye[name])
	plc_mutex.release()
	return result

def get_parameter(name):
	global plc_mutex, plc, plc_data_tpye
	plc_mutex.acquire()
	if type(name) == type([]):
		result = []
		for i in range(len(name)):
			result.append(plc.rad_by_name(name[i], plc_data_tpye[name[i]]))
	else:
		result = plc.read_by_name(name, plc_data_tpye[name])
	plc_mutex.release()
	return result

def get_status():
	global plc_mutex, plc
	plc_mutex.acquire()
	if not plc: #if connection doesn't bulid, return None.
		plc_mutex.release()
		return None
	status = PLCStatus()
	status.ServeON = plc.read_by_name(".bSLAM_ServeON", pyads.PLCTYPE_BOOL)
	status.right.bReady = plc.read_by_name(".SLAM_R[11]", pyads.PLCTYPE_DINT)
	status.right.bMoving = plc.read_by_name(".SLAM_R[12]", pyads.PLCTYPE_DINT)
	status.right.bError = plc.read_by_name(".SLAM_R[13]", pyads.PLCTYPE_DINT)
	status.right.pos = plc.read_by_name(".SLAM_R[14]", pyads.PLCTYPE_DINT)
	status.right.velocity = plc.read_by_name(".SLAM_R[15]", pyads.PLCTYPE_DINT)
	status.right.ErrorCode = plc.read_by_name(".SLAM_R[16]", pyads.PLCTYPE_DINT)
	status.right.temperature = plc.read_by_name(".SLAM_R[17]", pyads.PLCTYPE_DINT)
	status.right.volt = plc.read_by_name(".SLAM_R[18]", pyads.PLCTYPE_DINT)

	status.left.bReady = plc.read_by_name(".SLAM_L[11]", pyads.PLCTYPE_DINT)
	status.left.bMoving = plc.read_by_name(".SLAM_L[12]", pyads.PLCTYPE_DINT)
	status.left.bError = plc.read_by_name(".SLAM_L[13]", pyads.PLCTYPE_DINT)
	status.left.pos = plc.read_by_name(".SLAM_L[14]", pyads.PLCTYPE_DINT)
	status.left.velocity = plc.read_by_name(".SLAM_L[15]", pyads.PLCTYPE_DINT)
	status.left.ErrorCode = plc.read_by_name(".SLAM_L[16]", pyads.PLCTYPE_DINT)
	status.left.temperature = plc.read_by_name(".SLAM_L[17]", pyads.PLCTYPE_DINT)
	status.left.volt = plc.read_by_name(".SLAM_L[18]", pyads.PLCTYPE_DINT)
	plc_mutex.release()
	return status

def print_status(status):
	print("ServeON: ", status.ServeON)
	print("Right:")
	print("\tbReady: ", status.right.bReady)
	print("\tbMoving: ", status.right.bMoving)
	print("\tbError: ", status.right.bError)
	print("\tpos: ", status.right.pos)
	print("\tvelocity: ", status.right.velocity)
	print("\tErrorCode: ", status.right.ErrorCode)
	print("\ttemperature: ", status.right.temperature)
	print("\tvolt: ", status.right.volt)
	print("Left")
	print("\tbReady: ", status.left.bReady)
	print("\tbMoving: ", status.left.bMoving)
	print("\tbError: ", status.left.bError)
	print("\tpos: ", status.left.pos)
	print("\tvelocity: ", status.left.velocity)
	print("\tErrorCode: ", status.left.ErrorCode)
	print("\ttemperature: ", status.left.temperature)
	print("\tvolt: ", status.left.volt)
