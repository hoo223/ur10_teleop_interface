#!/usr/bin/python
# -*- coding: utf8 -*- 

# mode
INIT = 'init'
TELEOP = 'teleop'
CONTROL = 'control'
RL = 'rl'

## standard library
import numpy as np
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


class inputTarget(object):
  def __init__(self, env, verbose=False, prefix=""):

    self.verbose = verbose
    self.prefix = prefix
    self.env = env

    # teleoperation variable
    self.pre_button = None
    self.joystick_command = np.zeros(7)
    self.joystick_command[6] = -0.1
    self.keyboard_command = np.zeros(7)
    self.keyboard_command[6] = -0.1
    self.speed_gain = 0.00024 # for input scale
    self.speed_level = 5 # 로봇 움직임 속도 - 1~10 단계

    # subscriber
    self.joystick_command_sub = rospy.Subscriber('joystick_command', Float64MultiArray, self.joystick_command_callback)
    self.keyboard_command_sub = rospy.Subscriber('keyboard_command', Float64MultiArray, self.keyboard_command_callback)

    # publisher
    self.delta_target_input_pub = rospy.Publisher("delta_target_input", Float64MultiArray, queue_size= 10)


  def input_conversion(self, command):

    # joystick action to input mapping
    if self.env == 'unity':
      x_input = command[1]
      y_input = command[0]
      z_input = command[2]
      yaw_input = command[5]
    else:
      x_input = command[1]
      y_input = command[0]
      z_input = -command[2]
      yaw_input = -command[5]
    roll_input = pitch_input = 0
    
    button = int(command[6])
    if button == 1:
      roll_input = 1
    elif button == 2:
      roll_input = -1
    if button == 0:
      pitch_input = 1
    elif button == 3:
      pitch_input = -1
                 
    # change speed
    if button != self.pre_button: # restrict the continuous change
      if button == 5:
        self.speed_level += 1
        if self.speed_level > 10:
          self.speed_level = 10
      elif button == 4:
        self.speed_level -= 1
        if self.speed_level < 1:
          self.speed_level = 1
      self.pre_button = button

    input_scale = self.speed_gain * self.speed_level
    
    delta_target = []
    delta_target.append(x_input * input_scale)
    delta_target.append(y_input * input_scale)
    delta_target.append(z_input * input_scale)
    delta_target.append(roll_input * input_scale)
    delta_target.append(pitch_input * input_scale)
    delta_target.append(yaw_input * input_scale)

    return delta_target

  def reset_target(self, req):
    self.target_pose = self.init_pose
    res = TriggerResponse()
    res.message = "target pose reset"
    res.success = True
    return res

  def joystick_command_callback(self, data):
    self.joystick_command = data.data
    
  def keyboard_command_callback(self, data):
      self.keyboard_command = data.data

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
  args = rospy.myargv()
  if len(args) > 1: 
    prefix = '/'+args[1]
  else:
    prefix = ''
      
  # Get params
  env = rospy.get_param(prefix+"/env")
  init_pose = rospy.get_param(prefix+"/init_pose")
  init_joint_states = rospy.get_param(prefix+"/init_joint_states")
  
  # Node initialization
  rospy.init_node("input_target", anonymous=True)
  
  # Class instantiation
  it = inputTarget(env, prefix=prefix)
  
  # Set loop period
  rate = rospy.Rate(250) 
  
  # Loop
  while not rospy.is_shutdown():
    mode = rospy.get_param(prefix+"/mode")
    if mode == INIT:
      pass
      #print("target_pose initialized")
    elif mode == TELEOP:
      delta_target_joystick = it.input_conversion(it.joystick_command)
      delta_target_keyboard = it.input_conversion(it.keyboard_command)
      # combine delta target
      delta_target_input = Float64MultiArray()
      delta_target_input.data.append(delta_target_joystick[0]+delta_target_keyboard[0])
      delta_target_input.data.append(delta_target_joystick[1]+delta_target_keyboard[1])
      delta_target_input.data.append(delta_target_joystick[2]+delta_target_keyboard[2])
      delta_target_input.data.append(delta_target_joystick[3]+delta_target_keyboard[3])
      delta_target_input.data.append(delta_target_joystick[4]+delta_target_keyboard[4])
      delta_target_input.data.append(delta_target_joystick[5]+delta_target_keyboard[5])
      #print("delta_target calculated")
      #print(delta_target_input.data)
      it.delta_target_input_pub.publish(delta_target_input)
    rate.sleep()

if __name__ == '__main__':
  main()