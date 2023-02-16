#!/usr/bin/python
# -*- coding: utf8 -*- 


## standard library
import numpy as np
import time
from move_group_python_interface import MoveGroupPythonInteface
## standard library
import sys
#print(sys.executable) # python version
import copy

## ros library
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import Float64MultiArray, Float64
from tf.transformations import quaternion_from_euler
from ur10_teleop_interface.srv import SolveIk

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RL = 4
MOVEIT = 5
IDLE = 6
RESET = 7

class Traj2Target(object):

    def __init__(self):
        self.real = True
        self.unity = False
        
        self.init_sim_target = [0.0, 5.0]
        self.sim_target = copy.deepcopy(self.init_sim_target)
        q = quaternion_from_euler(0.052, 1.544, 3.114)
        self.target_orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.init_bot_target = self.sim2bot_conversion(self.init_sim_target)
        self.real_ik_result_pub = rospy.Publisher('/real/ik_result', Float64MultiArray, queue_size=10)
        
        self.sim_target_sub = rospy.Subscriber('sim_target', Float64MultiArray, self.sim_target_callback)

    def sim_target_callback(self, data):
        self.sim_target = data.data

    def sim2bot_conversion(self, sim_target):
        ps = Pose()
        ps.position.x = -0.0568*sim_target[1] -0.384
        ps.position.y = 0.0681*sim_target[0] -0.232
        ps.position.z = 0.244
        ps.orientation = self.target_orientation
        return ps

    def ik_solver(self, target_pose):
        if self.unity:
            rospy.wait_for_service('/unity/solve_ik')
            try:
                solve_ik = rospy.ServiceProxy('/unity/solve_ik', SolveIk)
                res = solve_ik(target_pose)
                return res
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)    
        if self.real:
            rospy.wait_for_service('/real/solve_ik')
            try:
                solve_ik = rospy.ServiceProxy('/real/solve_ik', SolveIk)
                res = solve_ik(target_pose)
                return res
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)                
        
    def solve_ik_by_moveit(self, target_pose):
        result = self.ik_solver(target_pose)
        ik_result = Float64MultiArray()
        if result.success:# IK가 성공하면 결과를 저장
            ik_result.data = [result.ik_result.data[0], result.ik_result.data[1], result.ik_result.data[2], result.ik_result.data[3], result.ik_result.data[4], result.ik_result.data[5]] 
            return ik_result
        else: # IK가 실패하면 teleop 정지
            print("ik failed")
            #rospy.set_param(self.prefix+'/teleop_state', "stop")

        print(rospy.get_param('/real/mode'))
    def check_ik_solution(self, target_pose):
        result = self.ik_solver(target_pose)
        if result.success:# IK가 성공하면 결과를 저장
            _ik_result = Float64MultiArray()
            _ik_result.data = [result.ik_result.data[0], result.ik_result.data[1], result.ik_result.data[2], result.ik_result.data[3], result.ik_result.data[4], result.ik_result.data[5]] 
            return(_ik_result)
        else: # IK가 실패하면 teleop 정지
            print("ik failed")
            #rospy.set_param(self.prefix+'/teleop_state', "stop")
            return False
    

def main():
    rospy.init_node('traj2target', anonymous=True)
    rospy.set_param('/real/mode', INIT)
    time.sleep(2)
    t2t = Traj2Target()
    rate = rospy.Rate(250)
    
    

    initialised = False

    while not rospy.is_shutdown():
        # print(rospy.get_param('/real/mode'))
        if rospy.get_param('/real/mode') == JOINT_CONTROL:
            initialised = False

            target_pose = t2t.sim2bot_conversion(t2t.sim_target)
            # print(target_pose.position)
            target_pose = t2t.solve_ik_by_moveit(target_pose)
            # print("target pose calculated")
            # print(target_pose.position)

            t2t.real_ik_result_pub.publish(target_pose)

        elif rospy.get_param('/real/mode') == INIT:
            if not initialised:
                # target_pose = copy.deepcopy(t2t.init_bot_target)
                # target_pose = t2t.solve_ik_by_moveit(target_pose)
                # t2t.real_ik_result_pub.publish(target_pose)
                time.sleep(2)
                print("target pose initialised")
                initialised = True

        rate.sleep()

if __name__ == '__main__':
    main()