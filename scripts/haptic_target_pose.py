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
from control_msgs.msg import GripperCommandActionGoal
from omni_msgs.msg import OmniButtonEvent
# from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input  as inputMsg
# from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg


#from cv_bridge import CvBridge

## custom library
from move_group_python_interface import MoveGroupPythonInteface

class hapticTargetPose(object):
  def __init__(self, prefix="", verbose=False):

    # Get params
    self.prefix = prefix
    self.env = rospy.get_param(prefix+"/env")
    self.haptic_feedback = rospy.get_param(prefix+"/haptic_feedback")
    self.init_joint_states = rospy.get_param(prefix+"/init_joint_states")

    # debugging
    self.verbose = verbose    

    # variables initialization
    self.grey_button_state = 0
    self.white_button_state = 0
    self.haptic_move_state = False
    self.white_button_pressed = False
    self.grey_button_pressed = False
    self.both_button_pressed = False
    self.gripper_closed = False
    self.start_target_pos = [0, 0, 0]
    self.haptic_scale_pos = 2 # translation scale factor
    self.haptic_scale_ori = 1 # translation scale factor
    self.delta_target_haptic = np.zeros(6)
    self.current_joint_states = copy.deepcopy(self.init_joint_states)
    self.wrist_1_joint = self.init_joint_states[3]
    self.wrist_2_joint = self.init_joint_states[4]
    self.wrist_3_joint = self.init_joint_states[5]

    # publisher
    self.haptic_target_pose_pub = rospy.Publisher(prefix+"/haptic_target_pose", Float64MultiArray, queue_size= 10)
    # self.haptic_error_pub = rospy.Publisher(prefix+"haptic_error", Float64MultiArray, queue_size=10)
    # self.haptic_rpy_pub = rospy.Publisher(prefix+"haptic_rpy", Float64MultiArray, queue_size=10)

    # subscriber
    self.with_gripper = True
    self.haptic_pose_sub = rospy.Subscriber(prefix+'/device1/pose', PoseStamped, self.haptic_pose_callback)
    self.haptic_joint_states_sub = rospy.Subscriber(prefix+'/device1/joint_states', JointState, self.haptic_joint_states_callback)
    self.haptic_button_sub = rospy.Subscriber(prefix+'/device1/button', OmniButtonEvent, self.haptic_button_callback)
    self.target_pose_sub = rospy.Subscriber(prefix+'/target_pose', PoseStamped, self.target_pose_callback)

    # tf listener
    self.listener = tf.TransformListener()
    
  def target_pose_callback(self, data):
    self.target_pose = data.pose

  # '/phantom/pose' topic 수신 시 콜백 - 햅틱 장치의 pose를 end-effector target pose로 변환
  def haptic_pose_callback(self, data):
    # 햅틱 장치 pose 얻기
    self.haptic_pose = data.pose
  
  def haptic_joint_states_callback(self, data):
    self.haptic_joint_states = data.position
    self.wrist_1_joint = -1.3006231672108264 + self.haptic_joint_states[4] - (-2.418408261237553) + 0.3
    self.wrist_2_joint = 1.5698880420405317 - (self.haptic_joint_states[3] - (3.1312592896916946))
    self.wrist_3_joint = 0.09116100519895554 - (self.haptic_joint_states[5] - (-3.061161795752593))     
  
  # '/phantom/button' topic 수신 시 콜백 - 햅틱 장치 버튼에 관한 동작 수행
  def haptic_button_callback(self, data):
    # white 버튼 
    if (self.white_button_pressed == False) and (data.white_button == 1):
      #print("white button pressed")
      # 시작시 햅틱 장치의 pose 저장
      self.start_haptic_pose = self.haptic_pose
      self.start_target_pos = [self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z]
      self.white_button_pressed = True
    elif (self.white_button_pressed == True) and (data.white_button == 0): # white 버튼을 안누르고 있으면 haptic move state 해제
      #print("white button released")
      self.white_button_pressed = False
    # grey 버튼
    if (self.grey_button_pressed == False) and (data.grey_button == 1):
      #print("grey button pressed")
      self.start_haptic_pose = self.haptic_pose
      self.start_target_ori = [self.target_pose.orientation.x, self.target_pose.orientation.y, self.target_pose.orientation.z, self.target_pose.orientation.w]
      #print(self.start_target_ori)
      self.grey_button_pressed = True
    elif (self.grey_button_pressed == True) and (data.grey_button == 0): 
      #print("grey button released")
      self.grey_button_pressed = False
    # white + grey 버튼
    if (self.both_button_pressed == False) and ((data.white_button == 1) and (data.grey_button == 1)):
      #print("both button pressed")
      self.both_button_pressed = True
      self.white_button_pressed = False
      self.grey_button_pressed = False
    elif (self.both_button_pressed == True) and ((data.white_button == 0) and (data.grey_button == 0)): 
      #print("both button released")
      self.both_button_pressed = False
    #print(data, self.white_button_pressed, self.grey_button_pressed, self.both_button_pressed)
  
  def init_mode(self):
    self.wrist_1_joint = self.init_joint_states[3]
    self.wrist_2_joint = self.init_joint_states[4]
    self.wrist_3_joint = self.init_joint_states[5]

  def teleop_mode(self):
    # white 버튼을 누르고 있는 동안에만 
    if self.white_button_pressed == True:
      self.translational_move()
    elif self.grey_button_pressed == True:
      #self.rotational_move()
      pass
    elif self.both_button_pressed == True:
      #self.gripper_action()
      pass

  def translational_move(self):
    x_error = self.haptic_pose.position.x - self.start_haptic_pose.position.x
    y_error = self.haptic_pose.position.y - self.start_haptic_pose.position.y
    z_error = self.haptic_pose.position.z - self.start_haptic_pose.position.z

    self.delta_target_haptic[0] = self.haptic_scale_pos*x_error
    self.delta_target_haptic[1] = self.haptic_scale_pos*y_error
    self.delta_target_haptic[2] = self.haptic_scale_pos*z_error

    haptic_target_pose = Float64MultiArray()
    haptic_target_pose.data.append(self.start_target_pos[0] + self.delta_target_haptic[0])
    haptic_target_pose.data.append(self.start_target_pos[1] + self.delta_target_haptic[1])
    haptic_target_pose.data.append(self.start_target_pos[2] + self.delta_target_haptic[2])
    self.haptic_target_pose_pub.publish(haptic_target_pose)

  def rotational_move(self):
    pass

  def gripper_action(self):
    if self.env == 'gazebo':
      pass
    elif self.env == 'unity':
      pass
    elif self.env == 'real':
      pass
     
def main():
  args = rospy.myargv()
  if len(args) > 1: 
    prefix = '/'+args[1]
  else:
    prefix = ''

  rospy.init_node("haptic_target_pose", anonymous=True) # Node initialization
  ht = hapticTargetPose(prefix=prefix) # Class instantiation
  rate = rospy.Rate(250) # Set loop period
  
  while not rospy.is_shutdown():
    mode = rospy.get_param(prefix+"/mode") 
    if mode == TELEOP:
      ht.teleop_mode()
    else:
      ht.init_mode()
    rate.sleep()

if __name__ == '__main__':
  main()