#!/usr/bin/python
# -*- coding: utf8 -*- 


import numpy as np
import time
from collections import defaultdict
from move_group_python_interface import MoveGroupPythonInteface

## ros library
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler


class GenerateOfflineTrajectory(object):
    """Joystick Controller for Lunar Lander."""

    def __init__(self, thread_rate, MoveGroup, real = True, unity = True, get_cur=True, get_next=True, get_desired=True, get_reward=False):
      self.thread_rate = thread_rate
      self.rate = rospy.Rate(self.thread_rate) 
      self.MGPI = MoveGroup 
      self.real = real
      self.unity = unity
      self.get_cur = get_cur
      self.get_next = get_next
      self.get_desired = get_desired
      self.get_reward = get_reward


    def get_current_robot_pose(self):
        return self.MGPI.get_current_pose(rpy=True)

    def generate_random_cosine_trajectory_parameter(self,x0,xf,T):
        n = np.random.randint(1,3,6)
        amp = (x0-xf)/2
        bias = (x0+xf)/2
        freq = (2*n-1)*np.pi/T
        return amp, bias, freq

    def generate_cosine_trajectory(self,amp, bias, freq, duration):
        t = np.linspace(0,duration,int(duration*self.thread_rate))
        xt = amp*np.cos(t*freq)+bias
        vt = -amp*freq*np.sin(t*freq)
        at = -amp*freq**2*np.cos(t*freq)
        return t,xt,vt,at

    def generate_cosine_trajectories(self):
        
        index = 1
        orientation_range = 0.6
        xyz_range = 0.85
        xyz_offset = 0.5
        
        inner_range = 0.4
        inner_offset = 0.5
        initial_pose = self.get_current_robot_pose()
        r_offset = initial_pose[3]
        p_offset = initial_pose[4]
        y_offset = initial_pose[5]
    
        if index==0: # x = 0~1 , y = -1~1, z=-1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[0] = np.random.random(1) *inner_range + inner_offset
            xf[0] = np.random.random(1) *inner_range + inner_offset

            x0[2] = np.random.random(1) *xyz_range + inner_offset
            xf[2] = np.random.random(1) *xyz_range + inner_offset
            
            x0[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            x0[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            x0[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            xf[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            xf[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            xf[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            
        elif index ==1: # x = -1~1, y = 0~1, z = -1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[1] = np.random.random(1) *inner_range + inner_offset
            xf[1] = np.random.random(1) *inner_range + inner_offset
            
            x0[2] = np.random.random(1) *xyz_range + inner_offset
            xf[2] = np.random.random(1) *xyz_range + inner_offset
            
            x0[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            x0[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            x0[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            xf[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            xf[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            xf[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            
        elif index == 2: # x = -1~0, y = -1~1, z = -1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[0] = -np.random.random(1) *inner_range - inner_offset
            xf[0] = -np.random.random(1) *inner_range - inner_offset
            
            
            x0[2] = np.random.random(1) *xyz_range + inner_offset
            xf[2] = np.random.random(1) *xyz_range + inner_offset
                    
            x0[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            x0[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            x0[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            xf[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            xf[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            xf[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            
        elif index ==3: # x = -1~1, y = -1~0, z = -1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[1] = -np.random.random(1) *inner_range - inner_offset
            xf[1] = -np.random.random(1) *inner_range - inner_offset
            
            x0[2] = np.random.random(1) *xyz_range + inner_offset
            xf[2] = np.random.random(1) *xyz_range + inner_offset
            
            x0[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            x0[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            x0[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            xf[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            xf[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            xf[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
        
        duration = np.ones(6,dtype=int) * int(np.random.random(1)*3+5) # target trajectory는 5~7초 동안 이동함, total step = duration * thread_rate
        amp, bias, freq = self.generate_random_cosine_trajectory_parameter(x0,xf,duration)

        t,xt,xvt,xat = self.generate_cosine_trajectory(amp[0], bias[0], freq[0], duration[0])
        t,yt,yvt,yat = self.generate_cosine_trajectory(amp[1], bias[1], freq[1], duration[1])
        t,zt,zvt,zat = self.generate_cosine_trajectory(amp[2], bias[2], freq[2], duration[2])
        t,rxt,rxvt,rxat = self.generate_cosine_trajectory(amp[3], bias[3], freq[3], duration[3])
        t,ryt,ryvt,ryat = self.generate_cosine_trajectory(amp[4], bias[4], freq[4], duration[4])
        t,rzt,rzvt,rzat = self.generate_cosine_trajectory(amp[5], bias[5], freq[5], duration[5])

        return np.vstack((xt,yt,zt,rxt,ryt,rzt)), len(t)



    def generate_init_random_cosine_trajectory_parameter(self,x0,xf,T):
        amp = (x0-xf)/2
        bias = (x0+xf)/2
        freq = np.pi/T
        return amp, bias, freq

    def generate_init_cosine_trajectories(self,xf): # initial trajectory는 8초동안 이동함
        duration = np.ones(6,dtype=int) * 8
        x0 = self.get_current_robot_pose()    
        amp, bias, freq = self.generate_init_random_cosine_trajectory_parameter(np.asarray(x0),np.asarray(xf),duration)

        t,xt,xvt,xat = self.generate_cosine_trajectory(amp[0], bias[0], freq[0], duration[0])
        t,yt,yvt,yat = self.generate_cosine_trajectory(amp[1], bias[1], freq[1], duration[1])
        t,zt,zvt,zat = self.generate_cosine_trajectory(amp[2], bias[2], freq[2], duration[2])
        t,rxt,rxvt,rxat = self.generate_cosine_trajectory(amp[3], bias[3], freq[3], duration[3])
        t,ryt,ryvt,ryat = self.generate_cosine_trajectory(amp[4], bias[4], freq[4], duration[4])
        t,rzt,rzvt,rzat = self.generate_cosine_trajectory(amp[5], bias[5], freq[5], duration[5])

        return np.vstack((xt,yt,zt,rxt,ryt,rzt)), len(t)


    def generate_target_pose(self,traj):
        ik_traj = []
        for i,pose in enumerate(traj.transpose()):
            _pose = self.input_conversion_yh(pose)
            self.target_pose_pub.publish(_pose)
            #print("target_pose calculated")
            _ik_result=self.check_ik_solution(_pose)
            if _ik_result==False:
                return False
            else:
                ik_traj.append(_ik_result)
        return ik_traj

    def check_self_collision_trajectory(self,traj):
            
        for target_joint_states in traj:
            iscollide = self.check_self_collision(target_joint_states)
            if not iscollide:
                print('self collision occured, re-plan the trajectory')
                return False
            return True
        
    def solve_ik_by_moveit(self, target_pose):
        result = self.ik_solver(target_pose)
        ik_result = Float64MultiArray()
        if result.success:# IK가 성공하면 결과를 저장
            ik_result.data = [result.ik_result.data[0], result.ik_result.data[1], result.ik_result.data[2], result.ik_result.data[3], result.ik_result.data[4], result.ik_result.data[5]] 
            return ik_result
        else: # IK가 실패하면 teleop 정지
            print("ik failed")
            rospy.set_param(self.prefix+'/teleop_state', "stop")

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

    def input_conversion(self,point):
        q_new = quaternion_from_euler(point[3],point[4], point[5]) # roll, pitch, yaw
        #q_new = quaternion_from_euler(-1.545, 0.001, 1.480) # roll, pitch, yaw
        target_orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])

        ps = Pose()
        ps.position.x = point[0]
        ps.position.y = point[1]
        ps.position.z = point[2]
        ps.orientation = target_orientation
        return ps

    def get_dataset(self,dataset, target_pose, target_vel,target_acc):
        # state : pose , velocity
        if self.real:
            dataset['real_cur_pos'].append()
            dataset['real_cur_vel'].append()

        if self.unity:
            dataset['unity_cur_pos'].append()
            dataset['unity_cur_vel'].append()

        if self.get_desired:
            dataset['desired_cur_pos'].append(target_pose)
            dataset['desired_cur_vel'].append(target_vel)
            dataset['desired_cur_acc'].append(target_acc)

        if self.get_reward:
            dataset['reward'].append()

        return dataset

    def arrange_dataset(self, dataset):
        if self.get_next:
            if self.real:
                dataset['real_next_pos'] = dataset['real_cur_pos'][1:]
                dataset['real_next_vel'] = dataset['real_cur_vel'][1:]
            if self.unity:
                dataset['unity_next_pos'] = dataset['unity_cur_pos'][1:]
                dataset['unity_next_vel'] = dataset['unity_cur_vel'][1:]

        if self.get_cur:
            if self.real:
                dataset['real_cur_pos'] = dataset['real_cur_pos'][:-1]
                dataset['real_cur_vel'] = dataset['real_cur_vel'][:-1]
            if self.unity:
                dataset['unity_cur_pos'] = dataset['unity_cur_pos'][:-1]
                dataset['unity_cur_vel'] = dataset['unity_cur_vel'][:-1]
                
        if self.get_desired:
            dataset['desired_next_pos'] = dataset['desired_cur_pos'][1:]
            dataset['desired_next_vel'] = dataset['desired_cur_vel'][1:]
            dataset['desired_next_acc'] = dataset['desired_cur_acc'][1:]

            dataset['desired_cur_pos'] = dataset['desired_cur_pos'][:-1]
            dataset['desired_cur_vel'] = dataset['desired_cur_vel'][:-1]
            dataset['desired_cur_acc'] = dataset['desired_cur_acc'][:-1]

        if self.get_reward:
            dataset['reward'] = dataset['reward'][:-1]

        return dataset

    def start_data_collection(self, episode_num):
        datasets = []
        
        if self.real:
            rospy.set_param('real/mode', 'idle') # set velocity to zero
        if self.unity:
            rospy.set_param('unity/mode', 'idle')      

        for i in range(episode_num):
            dataset = defaultdict(list)
            # generating target trajectory for 5~7 seconds
            target_traj, target_traj_length = self.generate_cosine_trajectories()
            ik_target_traj = self.generate_target_pose(target_traj)
            if ik_target_traj == False:
                episode_num += 1
                continue
            print('success generating target trajectory')

            # generating initial trajectory for 8 seconds
            init_traj, init_traj_length = self.generate_init_cosine_trajectories(target_traj[:,0])
            ik_init_traj = self.generate_target_pose(init_traj)
            if ik_init_traj == False:
                episode_num += 1
                continue
            print('success generating initial trajectory')

            # waiting one second for ready
            print('wait one second before going to the initial pose')
            self.rate.sleep(1)
            
            if self.real:
                rospy.set_param('real/mode', 'ik_result')
            if self.unity:
                rospy.set_param('unity/mode', 'ik_result')

            print('change idle mode to velocity control mode (joint space)')
            print('going to the initial pose')

            for j in range(init_traj_length):
                target_pose = self.input_conversion(init_traj[:,j])
                target_pose = self.solve_ik_by_moveit(target_pose)

                if self.real:
                    self.real_ik_result_pub.publish(target_pose)
                if self.unity:
                    self.unity_ik_result_pub.publish(target_pose)

                self.rate.sleep(self.thread_rate)  
            print('arrived at the initial pose')

            self.rate.sleep(1)
            print('wait one second before going to the target pose')
            print('going to the target pose')

            for j in range(target_traj_length):
                target_pose = self.input_conversion(target_traj[:,j])
                target_pose = self.solve_ik_by_moveit(target_pose)

                if self.real:
                    self.real_ik_result_pub.publish(target_pose)
                if self.unity:
                    self.unity_ik_result_pub.publish(target_pose)

                # dataset is saved by task space format
                dataset = self.get_dataset(dataset, target_traj[:,j])

                self.rate.sleep(self.thread_rate)        
            dataset = self.arrange_dataset(dataset)
            datasets.append(dataset)

            if self.real:
                rospy.set_param('real/mode', 'idle') # set velocity to zero
            if self.unity:
                rospy.set_param('unity/mode', 'idle') 

            print('change velocity control mode (joint space) to idle mode, set velocity zero')

            self.rate.sleep(1)
            print('wait one second before generating new trajectory')

        return datasets

# question 1 : mode change에 따른 설계가 적절한지
# question 2 : dataset을 저장하기 위한 data를 불러오는 방식 -> topic?


def main():
    MGPI  = MoveGroupPythonInteface(base_controller="arm_controller", 
                                    velocity_controller="joint_group_vel_controller", 
                                    gym=True, # unpause 일때만 가능, gym 환경에서 사용할 경우 gym=True
                                    verbose=False,
                                    prefix='real',
                                    node_name='robot_interface_for_rl')

    gen_traj = GenerateOfflineTrajectory(thread_rate = 250, MoveGroup = MGPI, real = True, unity = True)
    datasets = gen_traj.start_data_collection(episode_num = 20)
  
if __name__ == '__main__':
    main()