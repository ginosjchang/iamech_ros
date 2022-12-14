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
+ /tf
+ /odom
### Subscrib topic
+ /cmd_vel
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
