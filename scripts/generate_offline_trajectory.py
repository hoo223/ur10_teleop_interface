#!/usr/bin/python
# -*- coding: utf8 -*- 


import numpy as np
import time
from move_group_python_interface import MoveGroupPythonInteface

## ros library
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler


class GenerateOfflineTrajectory(object):
    """Joystick Controller for Lunar Lander."""

    def __init__(self, episode_num, episode_length, prefix, thread_rate, MoveGroup):
      self.thread_rate = thread_rate
      self.episode_num = episode_num
      self.episode_length = episode_length
      self.MGPI = MoveGroup # MoveGroupPythonInteface(base_controller="arm_controller", 
                            #        velocity_controller="joint_group_vel_controller", 
                            #        gym=True, # unpause 일때만 가능, gym 환경에서 사용할 경우 gym=True
                            #        verbose=False,
                            #        prefix=prefix,
                            #        node_name='robot_interface_yh') 


    def get_current_robot_pose(self):
        return self.MGPI.get_current_pose(rpy=True)

    def generate_random_cosine_trajectory_parameter(self,x0,xf,T):
        n = np.random.randint(1,3,6)
        amp = (x0-xf)/2
        bias = (x0+xf)/2
        freq = (2*n-1)*np.pi/T
        return amp, bias, freq

    def generate_init_random_cosine_trajectory_parameter(self,x0,xf,T):
        amp = (x0-xf)/2
        bias = (x0+xf)/2
        freq = np.pi/T
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
            _ik_result=self.solve_ik_by_moveit_yh(_pose)
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

    def solve_ik_by_moveit_yh(self, target_pose):
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

    def start_data_collection(self):
        for i in range(self.episode_num):
            
def main():

    gen_traj = GenerateOfflineTrajectory()
    
    
    # prefix 사용하는게 나은지? 
    while not rospy.is_shutdown():
        real_mode = rospy.get_param("/real/mode")
        unity_mode = rospy.get_param("unity/mode")

        if real_mode == 'offline' and unity_mode == 'offline':
            step = 0
            if step ==0:
                target_pose = Float64MultiArray()
                # generate episode trajectory from current robot pose to target pose
                traj, episode_length = gen_traj.generate_cosine_trajectories()
                ik_traj = gen_traj.generate_target_pose(traj)
                if ik_traj == False:
                    continue
                print('success generating target trajectory')
                
                # generate go to initial trajectory from target pose to initial pose
                init_traj,_ = gen_traj.generate_init_cosine_trajectories(traj[:,0])
                init_ik_traj = gen_traj.generate_target_pose(init_traj)
                if init_ik_traj == False:
                    continue
                print('success generating initial trajectory')
            
        elif status == 'Init Pose':
            # go to initial pose
            
        elif status == 'Target Pose':
            
        if step==0: # episode 시작할 때 실행
        # change controller
        j2t.MGPI.change_to_base_controller('initial')
        rospy.set_param(j2t.prefix+'/teleop_state', 'stop')
        rospy.set_param('/teleop_state', 'stop') 
        #generate trajectory
        traj,episode_length = j2t.generate_cosine_trajectories()
        print('generate trajectory')
        # check exist ik solution
        ik_traj = j2t.generate_target_pose(traj)   # 전체 trajectory에 대한 ik를 풀어서 return
        if ik_traj==False: # ik 실패하면 다시 trajectory 생성하기 위해 step 0 부터 다시 시작
            continue
        print('success ik')
        # check self collision
        iscollide = j2t.check_self_collision_trajectory(ik_traj)
        if iscollide==False:
            continue
        #initial_traj = j2t.generate_initial_pose_trajectory(ik_traj[0])
        
        print('success collision check')
        
        #generate go to init trajectory
        print(traj[:,0])
        init_traj,_ = j2t.generate_init_cosine_trajectories(traj[:,0])
        init_ik_traj = j2t.generate_target_pose(init_traj)   # 전체 trajectory에 대한 ik를 풀어서 return
        if init_ik_traj==False: # ik 실패하면 다시 trajectory 생성하기 위해 step 0 부터 다시 시작
            continue
        print('success ik')
        # check self collision
        iscollide = j2t.check_self_collision_trajectory(init_ik_traj)
        if iscollide==False:
            continue
        
        step = step+1  
        j2t.MGPI.change_to_velocity_controller('target')
        rospy.set_param(j2t.prefix+'/teleop_state', 'start')      
        rospy.set_param('/teleop_state', 'start') 
        '''
        ## go to initial pose
        print('go to init pose')
        res = j2t.MGPI.go_to_init_state(joint_goal=ik_traj[0].data)
        if res==True:
            print('arrived init pose')
            
            #rate = rospy.Rate(0.2)
            rate.sleep() 
            #rate = rospy.Rate(250)
            step=step+1
            j2t.MGPI.change_to_velocity_controller('target')
            rospy.set_param(j2t.prefix+'/teleop_state', 'start')
            
        elif res==False:
            
            #rate = rospy.Rate(250)
            rate.sleep()    
        '''    
            
        elif step < 2000:

        target_pose = j2t.input_conversion_yh(init_traj[:,step])
        target_pose = j2t.solve_ik_by_moveit(target_pose)
        #target_pose = ik_traj[step-1]#[ik_traj[step-1500][0],ik_traj[step-1500][1],ik_traj[step-1500][2],ik_traj[step-1500][3],ik_traj[step-1500][4],ik_traj[step-1500][5]]
        j2t.ik_result_pub.publish(target_pose)
        #print(target_pose.data)

        step=step+1
        #print(str(step)+'  go to target pose')
        rate.sleep()  
        elif step==2000:
        rate = rospy.Rate(0.2)
        print('arrived init pose')
        rate.sleep() 
        rate = rospy.Rate(250)
        step=step+1
        print('go to target pose')
        else: # episode 끝날때까지 target pose 하나씩 sampling
        target_pose = j2t.input_conversion_yh(traj[:,step-2001])
        target_pose = j2t.solve_ik_by_moveit(target_pose)
        #target_pose = ik_traj[step-1]#[ik_traj[step-1500][0],ik_traj[step-1500][1],ik_traj[step-1500][2],ik_traj[step-1500][3],ik_traj[step-1500][4],ik_traj[step-1500][5]]
        j2t.ik_result_pub.publish(target_pose)
        #print(target_pose.data)
        if step-2001==episode_length-1:
            print('arrived target pose')
            step=0
        elif step-2001<episode_length-1:
            step=step+1
        #print(str(step)+'  go to target pose')
        rate.sleep()

if __name__ == '__main__':
    main()