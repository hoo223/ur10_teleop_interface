#!/usr/bin/python
# -*- coding: utf8 -*- 

# mode
INIT = 'init'
TELEOP = 'teleop'
CONTROL = 'control'
RL = 'rl'

## standard library
import numpy as np
import time
import os
import sys
import copy

## ros library
import rospy
import ros
from rospy.service import ServiceException
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Float64MultiArray, String, Header
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion, Pose

#from cv_bridge import CvBridge

## custom library
from move_group_python_interface import MoveGroupPythonInteface


class targetPose(object):
  def __init__(self, init_pose, init_joint_states, env, verbose=False, prefix=""):

    self.verbose = verbose
    self.prefix = prefix
    self.env = env

    # subscriber
    self.delta_target_input_sub = rospy.Subscriber('delta_target_input', Float64MultiArray, self.delta_target_input_callback)
    self.delta_target_haptic_sub = rospy.Subscriber('delta_target_haptic', Float64MultiArray, self.delta_target_haptic_callback)
    self.haptic_target_pose_sub = rospy.Subscriber('haptic_target_pose', Float64MultiArray, self.haptic_target_pose_callback)

    # publisher
    self.target_pose_pub = rospy.Publisher("target_pose", Pose, queue_size= 10)

    # tf listener
    self.listener = tf.TransformListener()
    
    # UR10 initial pose
    self.init_pose = init_pose
    #self.init_joint_states = init_joint_states
    #self.current_joints = copy.deepcopy(self.init_joint_states)
    self.target_pose = copy.deepcopy(self.init_pose) # input device에 의해 조작되는 end-effector target pose

    self.delta_target_input = np.zeros(6)
    self.delta_target_haptic = np.zeros(6)

    # reset target service
    self.reset_target_service = rospy.Service('reset_target_pose', Trigger, self.reset_target)


  def update_target_pose(self):
    
    self.target_pose[0] += self.delta_target_input[0]
    self.target_pose[1] += self.delta_target_input[1]
    self.target_pose[2] += self.delta_target_input[2]
    self.target_pose[3] += self.delta_target_input[3]
    self.target_pose[4] += self.delta_target_input[4]
    self.target_pose[5] += self.delta_target_input[5]

    # self.target_pose[0] += self.delta_target_haptic[0]
    # self.target_pose[1] += self.delta_target_haptic[1]
    # self.target_pose[2] += self.delta_target_haptic[2]
    # self.target_pose[3] += self.delta_target_haptic[3]
    # self.target_pose[4] += self.delta_target_haptic[4]
    # self.target_pose[5] += self.delta_target_haptic[5]
    
    # print
    if self.verbose:
      print("speed: {:d}, x: {:.3f}, y: {:.3f}, z: {:.3f}, roll: {:.3f}, pitch: {:.3f}, yaw: {:.3f} \
            ".format(self.speed_level, self.target_pose[0], self.target_pose[1], self.target_pose[2], \
                                      self.target_pose[3], self.target_pose[4], self.target_pose[5]))

    #Workspace limit
    # # if self.target_pose[0] < X_LOWER:
    # #   self.target_pose[0] = X_LOWER
    # # elif self.target_pose[0] > X_UPPER:
    # #   self.target_pose[0] = X_UPPER
    # # if self.target_pose[1] < Y_LOWER:
    # #   self.target_pose[1] = Y_LOWER
    # # elif self.target_pose[1] > Y_UPPER:
    # #   self.target_pose[1] = Y_UPPER
    # # if self.target_pose[2] < Z_LOWER:
    # #   self.target_pose[2] = Z_LOWER
    # # elif self.target_pose[2] > Z_UPPER:
    # #   self.target_pose[2] = Z_UPPER

    # # get IK of target cartesian pose = target joint values
    ps = Pose()
    ps.position.x = self.target_pose[0]
    ps.position.y = self.target_pose[1]
    ps.position.z = self.target_pose[2]
    q_new = quaternion_from_euler(self.target_pose[3], self.target_pose[4], self.target_pose[5]) # roll, pitch, yaw
    self.target_orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])
    ps.orientation = self.target_orientation
    self.ps = ps


  def reset_target(self, req):
    self.target_pose = self.init_pose
    res = TriggerResponse()
    res.message = "target pose reset"
    res.success = True
    return res

  def delta_target_input_callback(self, data):
    self.delta_target_input = data.data
    
  def delta_target_haptic_callback(self, data):
    self.delta_target_haptic = data.data

  def haptic_target_pose_callback(self, data):
    self.target_pose[0] = data.data[0]
    self.target_pose[1] = data.data[1]
    self.target_pose[2] = data.data[2]

  def current_joint_callback(self, data):
    current_joints = list(data.position)
    # gazebo에서 나온 joint states 순서가 바뀌어 있음
    # [elbow_joint, robotiq_85_left_knuckle_joint, shoulder_lift_joint, shoulder_pan_joint,wrist_1_joint, wrist_2_joint, wrist_3_joint] - 2 6 1 0 3 4 5 
    self.current_joints[0] = current_joints[2]
    self.current_joints[1] = current_joints[6]
    self.current_joints[2] = current_joints[1]
    self.current_joints[3] = current_joints[0]
    self.current_joints[4] = current_joints[3]
    self.current_joints[5] = current_joints[4]
    self.current_joints[6] = current_joints[5]

    
def main():
  # Get params
  prefix = rospy.get_param("prefix", "")
  env = rospy.get_param("env")
  init_pose = rospy.get_param("init_pose")
  init_joint_states = rospy.get_param("init_joint_states")
  
  # Node initialization
  rospy.init_node("input_target", anonymous=True)
  
  # Class instantiation
  tp = targetPose(init_pose, init_joint_states, env, prefix=prefix)
  
  # Set loop period
  rate = rospy.Rate(250) 
  
  # Loop
  while not rospy.is_shutdown():
    mode = rospy.get_param("mode")
    if mode == INIT:
      tp.target_pose = copy.deepcopy(tp.init_pose)
      print("target_pose initialized")
    elif mode == TELEOP:
      tp.update_target_pose()
      print("target_pose calculated")
      tp.target_pose_pub.publish(tp.ps)

    rate.sleep()

if __name__ == '__main__':
  main()