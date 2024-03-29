#!/usr/bin/python
# -*- coding: utf8 -*- 

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RL = 4
MOVEIT = 5
IDLE = 6

## standard library
import numpy as np
import time
import os
import sys
import matplotlib.pyplot as plt
#import gym
#import torch

## ros library
import rospy
import ros
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
#from cv_bridge import CvBridge

## custom library
from move_group_python_interface import MoveGroupPythonInteface


## Function Definition
# Ros Duration to Time
def Duration2Time(duration):
  return duration.secs + duration.nsecs*1e-9


class TeleopController(object):
  def __init__(self, env=False, rsa=False, verbose=False, prefix=''):

    self.prefix = prefix
    self.target_joints = np.zeros(6)
    self.current_joints = np.zeros(6)
    self.p_gain = rospy.get_param(prefix+"/task_p_gain", 5)
    self.d_gain = rospy.get_param(prefix+"/task_p_gain", 1)
    self.pre_joint_errors = np.zeros(6)
    self.joint_vel_msg = Float64MultiArray()  
    self.teleop_state = 'stop'

    # subscriber
    self.target_joint_sub = rospy.Subscriber(self.prefix+'/ik_result', Float64MultiArray, self.target_joint_callback)
    self.current_joint_sub = rospy.Subscriber(self.prefix+'/joint_states', JointState, self.current_joint_callback)

    # publisher
    velocity_name = prefix+'/joint_group_vel_controller/command'
      
    self.vel_pub = rospy.Publisher(velocity_name, Float64MultiArray, queue_size=10)

  def control_loop(self):
    try:
      # compute control input
      joint_errors = np.array(self.target_joints) - np.array(self.current_joints)
      self.joint_vel_msg.data = self.p_gain*joint_errors + self.d_gain*(joint_errors-self.pre_joint_errors)
      self.pre_joint_errors = joint_errors
      # self.joint_vel_msg.data[3] = 0
      # self.joint_vel_msg.data[4] = 0
      # self.joint_vel_msg.data[5] = 0
    except:
      self.joint_vel_msg.data = np.zeros(6)
    self.vel_pub.publish(self.joint_vel_msg)

  def stop(self):
    self.joint_vel_msg.data = np.zeros(6)
    self.vel_pub.publish(self.joint_vel_msg)

  def target_joint_callback(self, data):
    self.target_joints = data.data
    
  def current_joint_callback(self, data):  
    current_joints = list(data.position)
    # gazebo에서 나온 joint states 순서가 바뀌어 있음
    # [elbow_joint, robotiq_85_left_knuckle_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint] - 3 2 0 4 5 6 
    self.current_joints[0] = current_joints[3]
    self.current_joints[1] = current_joints[2]
    self.current_joints[2] = current_joints[0]
    self.current_joints[3] = current_joints[4]
    self.current_joints[4] = current_joints[5]
    self.current_joints[5] = current_joints[6]


# main function
def main():
  args = rospy.myargv()
  if len(args) > 1: 
    prefix = '/'+args[1]
  else:
    prefix = ''
  
  rospy.init_node("teleop_controller", anonymous=True)
  tc = TeleopController(prefix=prefix)
  rate = rospy.Rate(250)
  while not rospy.is_shutdown():
    mode = rospy.get_param(prefix+"/mode")
    if (mode == TELEOP) or (mode == TASK_CONTROL) or (mode == JOINT_CONTROL):
      tc.control_loop()
    else:
      tc.stop()
      
    rate.sleep()

if __name__ == '__main__':
  main()

