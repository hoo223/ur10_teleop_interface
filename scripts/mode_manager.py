#!/usr/bin/python
# -*- coding: utf8 -*- 

## standard library
import sys
#print(sys.executable) # python version
import copy
from math import *
#import pygame
import time
import numpy as np
from numpy.linalg import inv, det, svd, eig

## ros library
import rospy
from tf.transformations import *
from std_msgs.msg import String, Float64MultiArray, Float64, Bool
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from geometry_msgs.msg import Quaternion
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController

# mode
INIT = 'init'
TELEOP = 'teleop'
CONTROL = 'control'
RL = 'rl'


## class definition
class ModeManager(object):
  """MoveGroupPythonInteface"""
  def __init__(self, prefix='', base_controller='arm_controller', velocity_controller='joint_group_vel_controller'):
    super(ModeManager, self).__init__()

    self.button = 0.0
    self.prefix = prefix
    self.base_controller = base_controller
    self.velocity_controller = velocity_controller

    # subscriber
    joystick_command_sub = rospy.Subscriber('joystick_command', Float64MultiArray, self.joystick_command_callback)
    keyboard_command_sub = rospy.Subscriber('keyboard_command', Float64MultiArray, self.keyboard_command_callback)

    
    # service
    self.change_to_vel_controller_service = rospy.Service('change_to_vel_controller', Trigger, self.change_to_velocity_controller)
    self.change_to_base_controller_service = rospy.Service('change_to_base_controller', Trigger, self.change_to_base_controller)

  def start_teleop(self):
    res = self.change_to_velocity_controller("")
    print(res.message)
    return res.success 

  def init_pose(self):
    res = self.change_to_base_controller("")
    if res.success:
      print(res.message)
      reset_pose_service = rospy.ServiceProxy(self.prefix+'/reset_pose', Trigger)
      req = TriggerRequest()
      res = reset_pose_service(req)
      success = res.success
    return success
  
  # https://answers.ros.org/question/259022/switching-between-controllers-with-ros_control-controller_manager/
  def controller_change(self, current_controller, target_controller):
    rospy.wait_for_service(self.prefix+'/controller_manager/switch_controller')
    try:
        #create a handle for calling the service
        switch_controller = rospy.ServiceProxy(self.prefix+'/controller_manager/switch_controller', SwitchController)
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
    self.controller_change(self.base_controller, self.velocity_controller)
    #self.controller_change(self.base_controller, self.velocity_controller, mode="/unity")
    res = TriggerResponse()
    res.success = True
    res.message = "changed to velocity controller"
    return res

  def change_to_base_controller(self, req):
    print("come in")
    self.controller_change(self.velocity_controller, self.base_controller)
    #self.controller_change(self.velocity_controller, self.base_controller, mode="/unity")
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
    
  def m_index_callback(self, data):
    self.m_index = data.data
  
  def eigen_value_callback(self, data):
    self.eigen_value = data.data

  def self_collision_callback(self, data):
    self.self_collision = data.data
    
  def check_singularity(self):
    singularity = False
    
    m_index = self.m_index
    e_value = list(self.eigen_value)
    self_collision = self.self_collision

    e_value.append(m_index)
    e_value.sort()
    #print(e_value)

    # penalty for reaching singularity
    if e_value[0] < 0.03:
      singularity = True
    
    # penalty for self collision
    if self_collision == True:
      singularity = True
      
    return singularity


def main():
  # args = rospy.myargv()
  # if len(args) > 1: 
  #   prefix = '/'+args[1]
  # else:
  #   prefix = ''
    
  prefix = rospy.get_param("prefix", '')
  base_controller = rospy.get_param("base_controller", "arm_controller") # sim: "arm_controller" / real: "scaled_pos_joint_traj_controller"
  velocity_controller = rospy.get_param("velocity_controller", "joint_group_vel_controller") 
  
  interface_param = '/move_group_python_interface'

  rospy.init_node("mode_manager", anonymous=True)
  rate = rospy.Rate(1100)
  mm = ModeManager(prefix, base_controller, velocity_controller) 

  # wait for initializing the interface
  while rospy.get_param(prefix+interface_param) != 'ready':
    print(prefix+interface_param + " not ready")

  ## set init pose
  while not mm.init_pose():
    print("Failed to go to init state")
  print("Success to get init state!")

  while not rospy.is_shutdown(): 
    mode = rospy.get_param("mode")
    #print(mode)
    if mode == INIT:
      if mm.button == 7.0:
        mm.start_teleop()
        rospy.set_param('/mode', 'teleop')
    elif mode == TELEOP:
      if mm.button == 6.0:
        mm.init_pose()
        rospy.set_param('/mode', 'init')
    elif mode == CONTROL:
      pass
    elif mode == RL:
      pass
      
  # Reset for singularity 
      
    rate.sleep()
  print("Finished")


if __name__ == '__main__':
  main()
  