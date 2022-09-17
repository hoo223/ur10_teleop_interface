#!/usr/bin/python
# -*- coding: utf8 -*- 


import numpy as np
import time
from collections import defaultdict
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
# ik 푸는거 사라짐.. movegroup에서도 service 형태가 있었는데 없어졌고
# ik를 풀어서 pulish to teleop_controller

# movegroup 사용하는거 어케 사용하는지 모르겠음. prefix를 unity로 주면 중복되서 그런건지 안됨
# 그러면 movegroup은 사용하지 말고 pose, velocity, IK 기능만 따로 여기서 구현?
# joint state에 joint angle / angular velocity가 있으므로 joint space의 angular velocity에다가 jacobian 곱해서 task space velocity를 얻음?
# 

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RL = 4
MOVEIT = 5
IDLE = 6
RESET = 7

class GenerateOfflineTrajectory(object):
    """Joystick Controller for Lunar Lander."""

    def __init__(self, thread_rate, real = True, unity = True, get_cur=True, get_next=True, get_desired=True, get_reward=False):
      self.thread_rate = thread_rate
      self.rate = rospy.Rate(self.thread_rate) 
      self.real = real
      self.unity = unity
      self.get_cur = get_cur
      self.get_next = get_next
      self.get_desired = get_desired
      self.get_reward = get_reward
      self.unity_ik_result_pub = rospy.Publisher('/unity/ik_result', Float64MultiArray, queue_size=10)
      self.real_ik_result_pub = rospy.Publisher('/real/ik_result', Float64MultiArray, queue_size=10)
      
      
      self.unity_pose_sub = rospy.Subscriber('/unity/current_pose_rpy', Float64MultiArray, self.unity_pose_callback)
      self.unity_velocity_sub = rospy.Subscriber('/unity/task_velocity', Float64MultiArray, self.unity_velocity_callback)
      self.unity_m_index_sub = rospy.Subscriber('/unity/m_index', Float64, self.unity_m_index_callback)
      
      
      self.real_pose_sub = rospy.Subscriber('/real/current_pose_rpy', Float64MultiArray, self.real_pose_callback)
      self.real_velocity_sub = rospy.Subscriber('/real/task_velocity', Float64MultiArray, self.real_velocity_callback)
      self.real_m_index_sub = rospy.Subscriber('/real/m_index', Float64, self.real_m_index_callback)
    

    def arrange_orientation_data(self, pose):
        
        pose = np.asarray(pose)
        
        if pose[3] > self.initial_pose[3] + self.orientation_range:
            pose[3] = pose[3] - np.pi
            
        if pose[4] >self.initial_pose[4] + self.orientation_range: # 0.3은 orientation range 보다 조금 더 큰 값 
            pose[4] = pose[4] - np.pi
        
        if pose[5] > self.initial_pose[5] +self.orientation_range:
            pose[5] = pose[5] - np.pi
            
        if pose[3] < self.initial_pose[3] - self.orientation_range:
            pose[3] = pose[3] + np.pi  
              
        if pose[4] < self.initial_pose[4] - self.orientation_range: # 0.3은 orientation range 보다 조금 더 큰 값 
            pose[4] = pose[4] + np.pi
        
        if pose[5] < self.initial_pose[5] - self.orientation_range:
            pose[5] = pose[5] + np.pi
        
        return (pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]) 
        
    def unity_pose_callback(self, data):
        self.unity_pose = data.data
        
    def unity_velocity_callback(self, data):
        self.unity_velocity = data.data

    def unity_m_index_callback(self, data):
        self.unity_m_index = data.data    
         
    def real_pose_callback(self, data):        
        self.real_pose = data.data
              
    def real_velocity_callback(self, data):
        self.real_velocity = data.data

    def real_m_index_callback(self, data):
        self.real_m_index = data.data     
    
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
        
        self.orientation_range = 0.2
        orientation_range = 0.2
        xyz_range = 0.80
        xyz_offset = 0.5
        
        inner_range = 0.55
        inner_offset = 0.30
            
        r_offset = self.initial_pose[3]
        p_offset = self.initial_pose[4]
        y_offset = self.initial_pose[5]

        if self.index==0: # x = 0~1 , y = -1~1, z=-1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[0] = np.random.random(1) *inner_range + inner_offset
            xf[0] = np.random.random(1) *inner_range + inner_offset

            x0[2] = np.random.random(1) *inner_range + inner_offset # index 2는 높이 방향 0.3~1.15
            xf[2] = np.random.random(1) *inner_range + inner_offset
            
            x0[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            x0[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            x0[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            xf[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            xf[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            xf[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            
        elif self.index ==1: # x = -1~1, y = 0~1, z = -1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range # -0.80~0.80
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[1] = np.random.random(1) *inner_range + inner_offset # 0.30~0.85
            xf[1] = np.random.random(1) *inner_range + inner_offset
            
            x0[2] = np.random.random(1) *inner_range + inner_offset # 0.30~0.85
            xf[2] = np.random.random(1) *inner_range + inner_offset
            
            x0[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            x0[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            x0[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            xf[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            xf[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            xf[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            
        elif self.index == 2: # x = -1~0, y = -1~1, z = -1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[0] = -np.random.random(1) *inner_range - inner_offset
            xf[0] = -np.random.random(1) *inner_range - inner_offset
            
            
            x0[2] = np.random.random(1) *inner_range + inner_offset
            xf[2] = np.random.random(1) *inner_range + inner_offset
                    
            x0[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            x0[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            x0[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            xf[3] = np.random.random(1) *2*orientation_range - orientation_range +r_offset
            xf[4] = np.random.random(1) *2*orientation_range - orientation_range +p_offset
            xf[5] = np.random.random(1) *2*orientation_range - orientation_range +y_offset
            
        elif self.index ==3: # x = -1~1, y = -1~0, z = -1~1
            x0 = np.random.random(6) * 2*xyz_range - xyz_range
            xf = np.random.random(6) * 2*xyz_range - xyz_range
            x0[1] = -np.random.random(1) *inner_range - inner_offset
            xf[1] = -np.random.random(1) *inner_range - inner_offset
            
            x0[2] = np.random.random(1) *inner_range + inner_offset
            xf[2] = np.random.random(1) *inner_range + inner_offset
            
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
        
        return np.vstack((xt,yt,zt,rxt,ryt,rzt)), np.vstack((xvt,yvt,zvt,rxvt,ryvt,rzvt)), np.vstack((xat,yat,zat,rxat,ryat,rzat)), len(t)


    def generate_init_random_cosine_trajectory_parameter(self,x0,xf,T):
        amp = (x0-xf)/2
        bias = (x0+xf)/2
        freq = np.pi/T
        return amp, bias, freq

    def generate_init_cosine_trajectories(self, xf): # initial trajectory는 8초동안 이동함
        duration = np.ones(6,dtype=int) * 8
        if self.unity:
            x0 = self.unity_pose
        if self.real:
            x0 = self.real_pose
        
        
        x0 = self.arrange_orientation_data(x0)
        
        amp, bias, freq = self.generate_init_random_cosine_trajectory_parameter(np.asarray(x0),np.asarray(xf),duration)
        print(x0)
        print(xf)
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
            _pose = self.input_conversion(pose)
            #self.target_pose_pub.publish(_pose)
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
            self.real_pose = self.arrange_orientation_data(self.real_pose)
            dataset['real_cur_pos'].append(self.real_pose)
            dataset['real_cur_vel'].append(self.real_velocity)
            dataset['real_m_index'].append(self.real_m_index)

        if self.unity:
            self.unity_pose = self.arrange_orientation_data(self.unity_pose)
            dataset['unity_cur_pos'].append(self.unity_pose)
            dataset['unity_cur_vel'].append(self.unity_velocity)
            dataset['unity_m_index'].append(self.unity_m_index)

        if self.get_desired:
            dataset['desired_cur_pos'].append(target_pose)
            dataset['desired_cur_vel'].append(target_vel)
            dataset['desired_cur_acc'].append(target_acc)

        if self.get_reward:
            dataset['reward'].append('')

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
        if self.real:
            dataset['real_m_index'] = dataset['real_m_index'][:-1]
        if self.unity:
            dataset['unity_m_index'] = dataset['unity_m_index'][:-1]

        return dataset

    def generate_online_trajectory_and_go_to_init(self, index):
        self.index = index
        
        if self.real:
            rospy.set_param('/real/mode', IDLE) # set velocity to zero
        if self.unity:
            rospy.set_param('/unity/mode', IDLE)   
               
        if self.unity:
            self.initial_pose = self.unity_pose
        if self.real:
            self.initial_pose = self.real_pose
            
        success = False
        
        while not success:
            target_traj, traj_vel, traj_acc, target_traj_length = self.generate_cosine_trajectories()
            ik_target_traj = self.generate_target_pose(target_traj)
            if ik_target_traj == False:
                continue
            print('success generating target trajectory')

            # generating initial trajectory for 8 seconds
            init_traj, init_traj_length = self.generate_init_cosine_trajectories(target_traj[:,0])
            ik_init_traj = self.generate_target_pose(init_traj)
            if ik_init_traj == False:
                continue
            print('success generating initial trajectory')  
            success = True  
            
            if self.real:
                rospy.set_param('/real/mode', JOINT_CONTROL)
            if self.unity:
                rospy.set_param('/unity/mode', JOINT_CONTROL)

            for j in range(init_traj_length):
                target_pose = self.input_conversion(init_traj[:,j])
                target_pose = self.solve_ik_by_moveit(target_pose)

                if self.real:
                    self.real_ik_result_pub.publish(target_pose)
                if self.unity:
                    self.unity_ik_result_pub.publish(target_pose)

                self.rate.sleep()  
            print('arrived at the initial pose')

            time.sleep(1)
            
            if self.real:
                rospy.set_param('/real/mode', IDLE) # set velocity to zero
            if self.unity:
                rospy.set_param('/unity/mode', IDLE)   
        return target_traj, traj_vel, traj_acc, target_traj_length
    
    def start_data_collection(self, episode_num, index):

        datasets = []
        self.index = index
        if self.real:
            rospy.set_param('/real/mode', IDLE) # set velocity to zero
        if self.unity:
            rospy.set_param('/unity/mode', IDLE)      
        
        if self.unity:
            self.initial_pose = self.unity_pose
        if self.real:
            self.initial_pose = self.real_pose
            
        success_episode_count = 0
        while episode_num > success_episode_count:
            print('episode number: '+str(success_episode_count))
            dataset = defaultdict(list)
            # generating target trajectory for 5~7 seconds
            target_traj, traj_vel, traj_acc, target_traj_length = self.generate_cosine_trajectories()
            ik_target_traj = self.generate_target_pose(target_traj)
            if ik_target_traj == False:
                continue
            print('success generating target trajectory')

            # generating initial trajectory for 8 seconds
            init_traj, init_traj_length = self.generate_init_cosine_trajectories(target_traj[:,0])
            ik_init_traj = self.generate_target_pose(init_traj)
            if ik_init_traj == False:
                continue
            print('success generating initial trajectory')

            # waiting one second for ready
            print('wait one second before going to the initial pose')
            time.sleep(1)

            if self.real:
                rospy.set_param('/real/mode', JOINT_CONTROL)
            if self.unity:
                rospy.set_param('/unity/mode', JOINT_CONTROL)

            print('change idle mode to velocity control mode (joint space)')
            print('going to the initial pose')

            for j in range(init_traj_length):
                target_pose = self.input_conversion(init_traj[:,j])
                target_pose = self.solve_ik_by_moveit(target_pose)

                if self.real:
                    self.real_ik_result_pub.publish(target_pose)
                if self.unity:
                    self.unity_ik_result_pub.publish(target_pose)

                self.rate.sleep()  
            print('arrived at the initial pose')

            time.sleep(1)
            print('wait one second before going to the target pose')
            print('going to the target pose')
            current = time.time()
            for j in range(target_traj_length):
        
                target_pose = self.input_conversion(target_traj[:,j])
                target_pose = self.solve_ik_by_moveit(target_pose)
                
                if self.real:
                    self.real_ik_result_pub.publish(target_pose)
                if self.unity:
                    self.unity_ik_result_pub.publish(target_pose)

                # dataset is saved by task space format
                dataset = self.get_dataset(dataset, target_traj[:,j], traj_vel[:,j],traj_acc[:,j])
                self.rate.sleep()
            end = time.time()
            print(end-current)
            print(target_traj_length/self.thread_rate)
            dataset = self.arrange_dataset(dataset)
            datasets.append(dataset)
            time.sleep(1)
            if self.real:
                rospy.set_param('/real/mode', IDLE) # set velocity to zero
            if self.unity:
                rospy.set_param('/unity/mode', IDLE) 

            print('change velocity control mode (joint space) to idle mode, set velocity zero')
            success_episode_count += 1
            
            print('wait one second before generating new trajectory')

        return datasets


def main():
    rospy.init_node("gen_traj", anonymous=True)
    rospy.set_param('/unity/mode', INIT)
    rospy.set_param('/real/mode', INIT)
    time.sleep(5)
    rate = rospy.Rate(1)
    gen_traj = GenerateOfflineTrajectory(thread_rate = 40, real = True, unity = True)
    rate.sleep()
    datasets = gen_traj.start_data_collection(episode_num = 5, index = 1)
    path = '/root/share/catkin_ws/src/ur10_teleop_interface/scripts/'
    filename = 'ntraj50_params_ori02_xyz_08_05_in_055_03.npy'
    np.save(path+filename,datasets)
    
    
    rospy.set_param('/unity/mode', INIT)
    rospy.set_param('/real/mode', INIT)
    time.sleep(5)
    
if __name__ == '__main__':
    main()