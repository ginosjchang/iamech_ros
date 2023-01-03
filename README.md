# iamech_ros
## Software
+ ROS Noetic on ubuntu 20.04
## Hardware
+ iAmech AGV
+ backoff PLC
## Usage Instructions
### Start the driver node
```bash=
roslaunch iamech_ros driver.launch
```
### Publish topic
+ /odom(nav_msgs/Odometry)
+ /plc_status(iamech_ros/PLCStatus)
### Subscrib topic
+ /cmd_vel(geometry_msgs/Twist)
### Service
+ ~set_ServeON(iamech_ros/ServeON)  
  set the serveON is 1 or 0.
+ ~get_position(iamech_ros/OdomPos)  
  get the current odometry position.
+ ~set_position(iamech_ros/OdomPos)  
  set the current odometry position.
### Parameters
+ yaml_path: The path of .yaml file which is the config of AVG and PLC in script/.
### Yaml Parameters
+ AGV
  + radius: mm
  + wheelbase: mm
+ PLC
  + ams_id
  + fps
  + hostname
  + ip
  + password
  + route_name
  + sender_ams
  + username
