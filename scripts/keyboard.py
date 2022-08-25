#!/usr/bin/python
# -*- coding: utf8 -*- 
import pygame
import numpy as np
import sys
import time
import rospy
from std_msgs.msg import Float64MultiArray

class KeyboardActor(object):

    def __init__(self, fps=1000):
        """Init."""
        self.human_agent_action = np.array([[0., 0.], [0., 0.]], dtype=np.float32)  # noop
        self.button = np.array([0], dtype=np.int32)
        pygame.init()
        self.t = None
        self.fps = fps

    def _get_human_action(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            self.human_agent_action[0, 0] = 1.0
        elif keys[pygame.K_s]:
            self.human_agent_action[0, 0] = -1.0
        else:
            self.human_agent_action[0, 0] = 0.0
            
        if keys[pygame.K_a]:
            self.human_agent_action[0, 1] = -1.0
        elif keys[pygame.K_d]:
            self.human_agent_action[0, 1] = 1.0
        else:
            self.human_agent_action[0, 1] = 0.0
            
        if keys[pygame.K_e]:
            self.human_agent_action[1, 0] = 1.0
        elif keys[pygame.K_c]:
            self.human_agent_action[1, 0] = -1.0
        else:
            self.human_agent_action[1, 0] = 0.0
            
        if keys[pygame.K_1]:
            self.button[0] = 6.0 # init mode
        elif keys[pygame.K_2]:
            self.button[0] = 7.0 # teleop mode
        else:
            self.button[0] = -1
        
        if abs(self.human_agent_action[0, 0]) < 0.01:
            self.human_agent_action[0, 0] = 0.0
        if abs(self.human_agent_action[1, 0]) < 0.01:
            self.human_agent_action[1, 0] = 0.0
        return self.human_agent_action, self.button

    def __call__(self):
        """Act."""
        action, button = self._get_human_action()
        if self.t and (time.time() - self.t) < 1. / self.fps:
            st = 1. / self.fps - (time.time() - self.t)
            if st > 0.:
                time.sleep(st)
        self.t = time.time()
        return action, button

    def reset(self):
        self.human_agent_action[:] = 0.

if __name__ == '__main__':
    fps = 1100
    actor = KeyboardActor(fps=fps)
    pub = rospy.Publisher('keyboard_command', Float64MultiArray, queue_size=10)
    rospy.init_node('keyboard', anonymous=True)
    rate = rospy.Rate(fps)
    width, height = 640, 480
    screen = pygame.display.set_mode((width, height))
    while not rospy.is_shutdown():
        # get input from joystick
        input, button = actor()
        command = Float64MultiArray()
        command.data.append(input[0][0]) # x
        command.data.append(input[0][1]) # y
        command.data.append(input[1][0]) # z
        command.data.append(0) # roll
        command.data.append(0) # pitch
        command.data.append(input[1][1]) # yaw
        command.data.append(button) # button
        pub.publish(command)

        rate.sleep()