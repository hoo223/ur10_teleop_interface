#!/usr/bin/python
# -*- coding: utf8 -*- 

import numpy as np

## ros library
import rospy
from sensor_msgs.msg import JointState
#from std_msgs.msg import Float64MultiArray

class jointStateIntegrator(object):
  def __init__(self, prefix=""):
    
    self.prefix = prefix
    self.ur10_joint_states = JointState()
    self.gripper_joint_states = JointState()
    self.gripper_joint_states.position = [0.0]
    self.gripper_joint_states.velocity = [0.0]
    self.gripper_joint_states.effort = [0.0]
        
    # subscriber
    self.ur10_joint_states_sub = rospy.Subscriber('/ur10/joint_states', JointState, self.ur10_joint_states_callback)
    self.gripper_joint_states_sub = rospy.Subscriber('/gripper/joint_states', JointState, self.gripper_joint_states_callback)

    # publisher
    self.joint_state_pub = rospy.Publisher(prefix+"/joint_states", JointState, queue_size= 10)

  def ur10_joint_states_callback(self, data):
    # gazebo에서 나온 joint states 순서가 바뀌어 있음
    # [elbow_joint, shoulder_lift_joint, shoulder_pan_joint,wrist_1_joint, wrist_2_joint, wrist_3_joint] - 2 1 0 3 4 5 
    self.ur10_joint_states = data
    #print(self.ur10_joint_states.velocity[5])
    
  def gripper_joint_states_callback(self, data):
    # gazebo에서 나온 joint states 순서가 바뀌어 있음
    # [robotiq_85_left_knuckle_joint]
    self.gripper_joint_states = data
    #print(self.gripper_joint_states.velocity[0])

    
  def publish_joint_states(self):
    try:
      joint_states = JointState()
      joint_states.header.stamp = rospy.Time.now()
      joint_states.name = ["elbow_joint", "robotiq_85_left_knuckle_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
      joint_states.position = [self.ur10_joint_states.position[0], 
                              self.gripper_joint_states.position[0], 
                              self.ur10_joint_states.position[1], 
                              self.ur10_joint_states.position[2], 
                              self.ur10_joint_states.position[3], 
                              self.ur10_joint_states.position[4], 
                              self.ur10_joint_states.position[5]]
      joint_states.velocity = [self.ur10_joint_states.velocity[0], 
                              self.gripper_joint_states.velocity[0], 
                              self.ur10_joint_states.velocity[1], 
                              self.ur10_joint_states.velocity[2], 
                              self.ur10_joint_states.velocity[3], 
                              self.ur10_joint_states.velocity[4], 
                              self.ur10_joint_states.velocity[5]]
      joint_states.effort   = [self.ur10_joint_states.effort[0], 
                              0.0, 
                              self.ur10_joint_states.effort[1], 
                              self.ur10_joint_states.effort[2], 
                              self.ur10_joint_states.effort[3], 
                              self.ur10_joint_states.effort[4], 
                              self.ur10_joint_states.effort[5]]
      self.joint_state_pub.publish(joint_states)
    except:
      print("error")
    
def main():
  args = rospy.myargv()
  if len(args) > 1: 
    prefix = '/'+args[1]
  else:
    prefix = ''
      
  # Get params
  env = rospy.get_param(prefix+"/env")

  # Node initialization
  rospy.init_node("joint_state_integrator", anonymous=True)
  
  jsi = jointStateIntegrator(prefix=prefix)
  
  # Set loop period
  rate = rospy.Rate(250) 
  
  # Loop
  while not rospy.is_shutdown():
    jsi.publish_joint_states()
    rate.sleep()

if __name__ == '__main__':
  main()