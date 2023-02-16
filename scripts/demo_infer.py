#!/usr/bin/python
# -*- coding: utf8 -*-


## standard library
import numpy as np
import time
import os
import sys
import copy

## PRO-GP requisites
from numpy.linalg import inv
import timeit
import time
from scipy.interpolate import interp1d

## ros library
import rospy
import ros
import tf
from std_msgs.msg import Float64MultiArray

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RL = 4
MOVEIT = 5
IDLE = 6
RESET = 7

# TF parameters
X_OFFSET = -0.449 # -0.446m
Y_OFFSET = -0.384 # -0.38m
Z_OFFSET = -0.01 # m

class TrajGenerator(object):
    def __init__(self, n_steps=200, total_steps=250, prefix=""):
        self.prefix = prefix

        # initialize GPR
        self.n_samples = 10
        self.n_steps = n_steps

        # load GP inference model
        self.model_folder = "/root/share/catkin_ws/src/ur10_teleop_interface/scripts/progp/Learned_Model_base1"
        self.known_envs = np.load(self.model_folder + '/known_envs.npy')
        self.known_mean_weights = np.load(self.model_folder + '/known_mean_weights.npy')
        self.known_var_weights = np.load(self.model_folder + '/known_var_weights.npy')
        self.mu = np.load(self.model_folder + '/mu.npy')
        self.noise_var = np.load(self.model_folder + '/noise_var.npy')
        self.max_length = np.load(self.model_folder + '/max_length.npy')
        self.block_PSI = np.load(self.model_folder + '/block_PSI.npy')
        self.num_gaba = np.load(self.model_folder + '/num_gaba.npy')

        # initialize viapoint position
        self.sim_viapt = [5.0, 8.0]
        self.sim_goal = [9.0, 5.0]

        # initialize current trajectory target
        self.init_target = [0.0, 5.0]
        self.traj_target = Float64MultiArray()
        self.traj_target.data = copy.deepcopy(self.init_target)

        # publisher
        self.traj_target_pub = rospy.Publisher("sim_target", Float64MultiArray, queue_size=10)

        # GP inference precomputation
        self.inv_Cbb_plus_noise_list = []
        self.yb_list = []
        self.infer_precomputation()

    def cam2sim_converter(self, cam_pt):
        x_sim = -(cam_pt[1] - 0.681) / 0.0681
        y_sim = cam_pt[0]/0.0568
        return [x_sim, y_sim]

    # Kernel function
    def kernel(self, x, y):
        return np.exp(np.matmul(-0.001*(x-y).T, (x-y)))  # squared exponential

    # Functions to compute the covariance matrix in the form Caa, Cab, Cba, Cbb
    def compute_cov_matrix1(self, known_environments):
        _Cbb = np.zeros((known_environments.shape[1], known_environments.shape[1]))
        for index1 in range(known_environments.shape[1]):
            _Cbb[index1, index1] = self.kernel(known_environments[:, index1], known_environments[:, index1])
            for index2 in range(index1 + 1, known_environments.shape[1]):
                _Cbb[index1, index2] = _Cbb[index2, index1] = self.kernel(known_environments[:, index1], known_environments[:, index2])
        return _Cbb
    
    def compute_cov_matrix2(self, new_environment, known_environments):
        _Caa = np.array(self.kernel(new_environment, new_environment)).reshape((1, 1))
        _Cab = np.zeros((1, known_environments.shape[1]))
        for known_env_index in range(known_environments.shape[1]):
            _Cab[0, known_env_index] = self.kernel(new_environment, known_environments[:, known_env_index])
        _Cba = _Cab.T
        return _Caa, _Cab, _Cba

    # precomputation to GP inference
    def infer_precomputation(self):
        Cbb = self.compute_cov_matrix1(self.known_envs)
        self.inv_Cbb_plus_noise_list = []
        self.yb_list = []
        for w_index in range(2*self.num_gaba):
            w_var_matrix = np.diag(self.known_var_weights[w_index, :])
            self.inv_Cbb_plus_noise_list.append(inv(Cbb + w_var_matrix))
            yb = self.known_mean_weights[w_index, :].reshape(self.known_envs.shape[1], 1)
            self.yb_list.append(yb)
        
    # Function to infer mean trajectory
    def infer_trajectory(self, viapt):
        sim_env = [viapt[0], viapt[1], self.sim_goal[0], self.sim_goal[1]]
        # GP inference
        new_mean_weights = np.zeros((2*self.num_gaba, 1))
        new_var_weights = np.zeros((2*self.num_gaba, 1))
        new_env = np.array(self.init_target + sim_env)
        for w_index in range(2*self.num_gaba):
            Caa, Cab, Cba = self.compute_cov_matrix2(new_env, self.known_envs)
            yb = self.yb_list[w_index]
            m = self.mu[w_index] + np.matmul(np.matmul(Cab, self.inv_Cbb_plus_noise_list[w_index]), (self.yb_list[w_index] - self.mu[w_index]))
            D = Caa + self.noise_var[w_index] - np.matmul(np.matmul(Cab, self.inv_Cbb_plus_noise_list[w_index]), Cba)
            new_mean_weights[w_index] = m
            new_var_weights[w_index] = D
        
        # compute mean trajectory and interpolate
        mean_traj = np.dot(new_mean_weights.T, np.transpose(self.block_PSI)).reshape(-1, )
        old_x = np.linspace(0, 1, np.int(mean_traj.size/2))
        new_x = np.linspace(0, 1, self.n_steps)
        interp = interp1d(old_x, mean_traj.reshape((2, self.max_length), order='C'))
        mean_traj = interp(new_x)
        return mean_traj

    
# main function
def main():
    rospy.init_node('traj_infer', anonymous=True)
    n_steps = 200
    tg = TrajGenerator(n_steps=n_steps)

    # transform broadcaster/listener to anchor & receive marker position
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    
    rate = rospy.Rate(20)
    rate.sleep()

    i = 0
    initialised = True

    while not rospy.is_shutdown():

        # table-to-robot base transform
        br.sendTransform((X_OFFSET, Y_OFFSET, Z_OFFSET),
                     tf.transformations.quaternion_from_euler(0, 0, -np.pi/2), # np.pi/2, 0, np.pi/2
                     rospy.Time.now(),
                     'real/base_link',
                     "marker_hole_frame")
        
        # table-to-robot base transform
        br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, np.pi/2),
                     rospy.Time.now(),
                     'viapt_marker',
                     "marker_hole_frame")
        
        # listen to viapt marker position - zeroed on marker_object_frame
        try:
            (trans, rot) = listener.lookupTransform('viapt_marker', 'marker_object_frame', rospy.Time(0))
            tg.sim_viapt = tg.cam2sim_converter(trans[0:2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if rospy.get_param('/real/mode') == JOINT_CONTROL:
            initialised = False
            if i < tg.max_length:
                viapt = tg.sim_viapt
                mean_traj = tg.infer_trajectory(viapt)

                tg.traj_target.data = [mean_traj[0,i], mean_traj[1,i]]
                tg.traj_target_pub.publish(tg.traj_target)
                print(tg.traj_target)

                i += 1
            else:
                print("trajectory inference complete")
                rospy.set_param('/real/mode', INIT)
        
        # elif rospy.get_param('/real/mode') == IDLE:
        #     tg.traj_target_pub.publish(tg.traj_target)

        elif rospy.get_param('/real/mode') == INIT:
            if not initialised:
                tg.traj_target_pub.publish(tg.traj_target)
                initialised = True
                print("ready to initialise")
                time.sleep(1)
            i = 0
            viapt = tg.sim_viapt
            tg.traj_target.data = tg.init_target
            tg.traj_target_pub.publish(tg.traj_target)

        rate.sleep()

if __name__ =='__main__':
    main()


