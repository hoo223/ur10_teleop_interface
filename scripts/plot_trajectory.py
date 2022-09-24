#!/usr/bin/python
# -*- coding: utf8 -*- 


import numpy as np
import time
from collections import defaultdict
import matplotlib.pyplot as plt




def main():
    path = '/root/share/catkin_ws/src/ur10_teleop_interface/scripts/'
    filename = 'ntraj50_params_ori02_xyz_08_05_in_055_03_trial2.npy'
    datasets = list(np.load(path+filename))
    
    print(np.asarray(datasets[45]['real_next_pos'])[0,0])
    print(np.asarray(datasets[46]['real_next_pos'])[0,0])
    del datasets[45]
    del datasets[37]
    del datasets[20]
    
    np.save(path+filename+'_2',datasets)
    
    '''
    i=37
    
    dataset = datasets[i]
    
    fig = plt.figure(figsize=(30,20))
    ax1 = fig.add_subplot(3,2,1)
    #ax1.set_ylim([-2,2])
    ax1.plot(np.asarray(dataset['real_next_pos'])[:,0], lw=3, color='r' , label='real_pos')
    #ax1.plot(np.asarray(dataset['unity_next_pos'])[:,0], lw=3, color = 'b', label='unity_pos')
    ax1.plot(np.asarray(dataset['desired_cur_pos'])[:,0], lw=3, color = 'k', label='desired_pos')
    
    ax2 = fig.add_subplot(3,2,2)
    #ax2.set_ylim([-2,2])
    ax2.plot(np.asarray(dataset['real_next_pos'])[:,1], lw=3, color='r' , label='real_pos')
    #ax2.plot(np.asarray(dataset['unity_next_pos'])[:,1], lw=3, color = 'b', label='unity_pos')
    ax2.plot(np.asarray(dataset['desired_cur_pos'])[:,1], lw=3, color = 'k', label='desired_pos')
    
    ax3 = fig.add_subplot(3,2,3)
    #ax3.set_ylim([-2,2])
    ax3.plot(np.asarray(dataset['real_next_pos'])[:,2], lw=3, color='r' , label='real_pos')
    #ax3.plot(np.asarray(dataset['unity_next_pos'])[:,2], lw=3, color = 'b', label='unity_pos')
    ax3.plot(np.asarray(dataset['desired_cur_pos'])[:,2], lw=3, color = 'k', label='desired_pos')
    
    ax4 = fig.add_subplot(3,2,4)
    #ax4.set_ylim([-np.pi,np.pi])
    ax4.plot(np.asarray(dataset['real_next_pos'])[:,3], lw=3, color='r' , label='real_pos')
    #ax4.plot(np.asarray(dataset['unity_next_pos'])[:,3], lw=3, color = 'b', label='unity_pos')
    ax4.plot(np.asarray(dataset['desired_cur_pos'])[:,3], lw=3, color = 'k', label='desired_pos')
    
    ax5 = fig.add_subplot(3,2,5)
    #ax5.set_ylim([-np.pi,np.pi])
    ax5.plot(np.asarray(dataset['real_next_pos'])[:,4], lw=3, color='r' , label='real_pos')
    #ax5.plot(np.asarray(dataset['unity_next_pos'])[:,4], lw=3, color = 'b', label='unity_pos')
    ax5.plot(np.asarray(dataset['desired_cur_pos'])[:,4], lw=3, color = 'k', label='desired_pos')
    
    ax6 = fig.add_subplot(3,2,6)
    #ax6.set_ylim([-np.pi,np.pi])
    ax6.plot(np.asarray(dataset['real_next_pos'])[:,5], lw=3, color='r' , label='real_pos')
    #ax6.plot(np.asarray(dataset['unity_next_pos'])[:,5], lw=3, color = 'b', label='unity_pos')
    ax6.plot(np.asarray(dataset['desired_cur_pos'])[:,5], lw=3, color = 'k', label='desired_pos')
    ax1.text(15,0,i)
    plt.legend(fontsize=15)
    plt.show()
    '''
    
if __name__ == '__main__':
    main()