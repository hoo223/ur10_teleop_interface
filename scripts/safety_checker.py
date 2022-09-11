#!/usr/bin/python
# -*- coding: utf8 -*- 

## ros library
import rospy
from std_msgs.msg import String, Float64MultiArray, Float64, Bool

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RSA = 4
MOVEIT = 5
IDLE = 6

## class definition
class SafetyChecker(object):
  def __init__(self, prefix=''):
    super(SafetyChecker, self).__init__()
    
    # variables
    self.safety_threshold = 0.03
    
    # subscriber
    self.m_index_sub = rospy.Subscriber(prefix+'/m_index', Float64, self.m_index_callback)
    self.eigen_value_sub = rospy.Subscriber(prefix+'/eigen_value', Float64MultiArray, self.eigen_value_callback)
    self.self_collision_sub = rospy.Subscriber(prefix+'/self_collision', Bool, self.self_collision_callback)
    
  def m_index_callback(self, data):
    self.m_index = data.data
    print(self.m_index)
  
  def eigen_value_callback(self, data):
    self.eigen_value = data.data

  def self_collision_callback(self, data):
    self.self_collision = data.data
    
  def check_singularity(self):
    singularity = False
    try:
      m_index = self.m_index
      e_value = list(self.eigen_value)
      self_collision = self.self_collision
      print("data is ready")

      e_value.append(m_index)
      e_value.sort()

      # penalty for reaching singularity
      if e_value[0] < self.safety_threshold:
        singularity = True
      
      # penalty for self collision
      if self_collision == True:
        singularity = True
    except:
      print("data is not ready")
      
    return singularity


def main():
  args = rospy.myargv()
  if len(args) > 1: 
    prefix = '/'+args[1]
  else:
    prefix = ''

  rospy.init_node("safety_checker", anonymous=True)
  rate = rospy.Rate(1100)
  sc = SafetyChecker(prefix=prefix) 
  
  while not rospy.is_shutdown(): 
    
    if sc.check_singularity():
      print("singularity!!")
      rospy.set_param(prefix+'/mode', INIT)
      
    rate.sleep()

if __name__ == '__main__':
  main()
  