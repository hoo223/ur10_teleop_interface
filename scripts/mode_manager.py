#!/usr/bin/python
# -*- coding: utf8 -*- 

## standard library
from math import *

## ros library
import rospy
from tf.transformations import *
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from move_group_python_interface import MoveGroupPythonInteface

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RSA = 4
MOVEIT = 5
IDLE = 6

## class definition
class ModeManager(object):
  """MoveGroupPythonInteface"""
  def __init__(self, prefix='', base_controller='arm_controller', velocity_controller='joint_group_vel_controller'):
    super(ModeManager, self).__init__()

    self.button = 0.0
    if prefix == "/real":
      self.move_group_prefix = "/real"
      self.controller_prefix = "/ur10"
    else:
      self.move_group_prefix = prefix
      self.controller_prefix = prefix
    self.base_controller = rospy.get_param(prefix+"/base_controller", prefix+"/arm_controller") # sim: "arm_controller" / real: "scaled_pos_joint_traj_controller"
    self.velocity_controller = rospy.get_param(prefix+"/velocity_controller", prefix+"/joint_group_vel_controller") 
    
    # subscriber
    joystick_command_sub = rospy.Subscriber('joystick_command', Float64MultiArray, self.joystick_command_callback)
    keyboard_command_sub = rospy.Subscriber('keyboard_command', Float64MultiArray, self.keyboard_command_callback)

    # service
    self.change_to_vel_controller_service = rospy.Service(self.controller_prefix+'change_to_vel_controller', Trigger, self.change_to_velocity_controller)
    self.change_to_base_controller_service = rospy.Service(self.controller_prefix+'change_to_base_controller', Trigger, self.change_to_base_controller)
  
  # https://answers.ros.org/question/259022/switching-between-controllers-with-ros_control-controller_manager/
  def controller_change(self, current_controller, target_controller):
    # print(self.controller_prefix+'/controller_manager/switch_controller')
    rospy.wait_for_service(self.controller_prefix+'/controller_manager/switch_controller')
    print(self.controller_prefix+'/controller_manager/switch_controller', " service ready")
    try:
        #create a handle for calling the service
        switch_controller = rospy.ServiceProxy(self.controller_prefix+'/controller_manager/switch_controller', SwitchController)
        # http://docs.ros.org/en/api/controller_manager_msgs/html/srv/SwitchController.html
        req = SwitchControllerRequest()
        req.start_controllers = [target_controller]
        req.stop_controllers = [current_controller]
        req.strictness = 1
        req.start_asap = False
        req.timeout = 0.0
        #req = SwitchControllerRequest(start_controllers=target_controller, stop_controllers=current_controller, strictness=1, 
        #                              start_asap=False, timeout=0.0)
        res = switch_controller(req)
        if res:
            print(res)
            print("controller changed from {} to {}".format(current_controller, target_controller))
            return True
        else:
            print("failed to change controller")
            return False
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e) 
        return False

  def change_to_velocity_controller(self, req):
    print("change to ", self.velocity_controller)
    self.controller_change(self.base_controller, self.velocity_controller)
    res = TriggerResponse()
    res.success = True
    res.message = "changed to velocity controller"
    return res

  def change_to_base_controller(self, req):
    print("change to ", self.base_controller)
    self.controller_change(self.velocity_controller, self.base_controller)
    res = TriggerResponse()
    res.success = True
    res.message = "changed to base controller"
    return res

  def joystick_command_callback(self, data):
    self.joystick_command = data.data
    self.button = self.joystick_command[6]
    
  def keyboard_command_callback(self, data):
    self.keyboard_command = data.data
    self.button = self.keyboard_command[6]


def main():
  args = rospy.myargv()
  if len(args) > 1: 
    prefix = '/'+args[1]
  else:
    prefix = ''

  rospy.init_node("mode_manager", anonymous=True)
  rate = rospy.Rate(1100)
  mm = ModeManager(prefix=prefix) 
  mgi = MoveGroupPythonInteface(prefix=prefix)
  
  ## set init pose
  mgi.init_pose()

  pre_mode = 0
  while not rospy.is_shutdown(): 
    # mode change by input command
    if mm.button == 6.0:
      rospy.set_param(prefix+'/mode', INIT)
    elif mm.button == 7.0:
      rospy.set_param(prefix+'/mode', TELEOP)

    # read current mode
    mode = rospy.get_param(prefix+"/mode")
    
    # mode change
    if mode is not pre_mode:
      if mode == INIT:
        print("INIT mode")
        mm.change_to_base_controller("")
        mgi.init_pose()
      elif mode == TELEOP :
        print("TELEOP mode")
        mm.change_to_velocity_controller("")
      elif mode == TASK_CONTROL:
        mm.change_to_velocity_controller("")
        print("TASK_CONTROL mode")
      elif mode == JOINT_CONTROL:
        mm.change_to_velocity_controller("")
        print("JOINT_CONTROL mode")
      elif mode == IDLE:
        mm.change_to_velocity_controller("")
        print("IDLE mode")
      elif mode == MOVEIT:
        mm.change_to_base_controller("")
        print("MOVEIT mode")
      elif mode == RSA:
        mm.change_to_velocity_controller("")
        print("RSA mode")
      pre_mode = mode

    rate.sleep()

if __name__ == '__main__':
  main()
  