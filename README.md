# ur10_teleop_interface
teleoperation package for ur10 manipulator

## Launch
* unity_bringup.launch
* unity_teleop.launch
* real_bringup.launch
* real_teleop.launch
* teleop_interface.launch

## PROGP demo initialisation
* roscore
* ur10 - ur10_tcp
* core - real_bringup
* ce - real_teleop
* realsense
* cam - roslaunch aruco_ros double.launch
* came - rosrun aruco_ros tf_connector_double_promp.py
* ce - roslaunch ur10_teleop_interface progp_demo.launch
* rosparam set /real/mode 3 (for joint control)

## Scripts
* 

## Src

## Include
