import yaml
import rospy

def yaml_read():
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
