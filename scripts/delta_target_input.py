#!/usr/bin/python
# -*- coding: utf8 -*- 

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RSA = 4
MOVEIT = 5
IDLE = 6

## standard library
import numpy as np

## ros library
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse

#from cv_bridge import CvBridge


class inputTarget(object):
  def __init__(self, env, verbose=False, prefix=""):

    self.verbose = verbose
    self.prefix = prefix
    self.env = env

    # teleoperation variable
    self.pre_button = None
    self.joystick_command = list(np.zeros(7))
    self.joystick_command[6] = -1.0
    self.keyboard_command = list(np.zeros(7))
    self.keyboard_command[6] = -1.0
    self.rsa_command = np.zeros(7)
    self.rsa_command[6] = -0.1
    self.input_pos_gain = rospy.get_param(prefix+"input_pos_gain", 0.00004)
    self.input_ori_gain = rospy.get_param(prefix+"input_ori_gain", 0.00008)
    self.speed_level = 5 # 로봇 움직임 속도 - 1~10 단계

    # subscriber
    self.joystick_command_sub = rospy.Subscriber('joystick_command', Float64MultiArray, self.joystick_command_callback)
    self.keyboard_command_sub = rospy.Subscriber('keyboard_command', Float64MultiArray, self.keyboard_command_callback)
    self.keyboard_command_sub = rospy.Subscriber('rsa_command', Float64MultiArray, self.rsa_command_callback)

    # publisher
    self.delta_target_input_pub = rospy.Publisher("delta_target_input", Float64MultiArray, queue_size= 10)


  def input_conversion(self, command):

    # joystick action to input mapping
    x_input = command[1]
    y_input = command[0]
    z_input = command[2]
    roll_input = command[3]
    pitch_input = command[4]
    yaw_input = command[5]
      
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
        self.speed_level += 0.1
        if self.speed_level > 20:
          self.speed_level = 20
      elif button == 4:
        self.speed_level -= 0.1
        if self.speed_level < 0:
          self.speed_level = 0
      self.pre_button = button

    position_scale = self.input_pos_gain * self.speed_level
    orientation_scale = self.input_ori_gain * self.speed_level
    
    delta_target = []
    delta_target.append(x_input * position_scale)
    delta_target.append(y_input * position_scale)
    delta_target.append(z_input * position_scale)
    delta_target.append(roll_input * orientation_scale)
    delta_target.append(pitch_input * orientation_scale)
    delta_target.append(yaw_input * orientation_scale)

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

  def rsa_command_callback(self, data):
    self.rsa_command = data.data
  
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
    if mode == TELEOP:
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
      it.delta_target_input_pub.publish(delta_target_input)
    elif mode == RSA:
      delta_target_rsa = it.input_conversion(it.rsa_command)
      # combine delta target
      delta_target_input = Float64MultiArray()
      delta_target_input.data.append(delta_target_rsa[0])
      delta_target_input.data.append(delta_target_rsa[1])
      delta_target_input.data.append(delta_target_rsa[2])
      delta_target_input.data.append(delta_target_rsa[3])
      delta_target_input.data.append(delta_target_rsa[4])
      delta_target_input.data.append(delta_target_rsa[5])
      it.delta_target_input_pub.publish(delta_target_input)
    rate.sleep()

if __name__ == '__main__':
  main()