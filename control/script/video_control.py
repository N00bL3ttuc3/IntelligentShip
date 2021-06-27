#!/usr/bin/env python3
"""
Received Control

Author: SheffieldWang
"""
#import basic
import math
import numpy as np


#import ROS
import rospy
from std_msgs.msg import Float64,Int64MultiArray
from control.msg import Command


K = 1 # weight of angel
max = 240
max_steer = np.radians(45.0)  # [rad] max steering angle


class VideoControlNode():
    def __init__(self):
        #ros
        self.control_sub_ = rospy.Subscriber("control_msg", Int64MultiArray, self.callbackFromControlmsg)
        self.postion_sub_ = rospy.Subscriber("dis", Float64, self.callbackFromPostionmsg)
        self.command_pub_ = rospy.Publisher("video_control_command",Command,queue_size=1)
        #command
        self.command_ = Command()
        self.dl_ = 0.0
        self.ai_ = 0.0
        self.flag_ = 0


    def run(self):
        rospy.spin()

    def callbackFromControlmsg(self,controlmsg):
        print(controlmsg.data)
        #assert sum(controlmsg.data) == 1,"the sum of control_msg.data is not equal to 1"
        if controlmsg.data[2] == 1:
            self.flag_ = 1
        elif controlmsg.data[3] == 1:
            self.flag_ = 0
        self.publishCommand()
        

    def callbackFromPostionmsg(self,msg):
        self.dl_ =-msg.data/max*max_steer*K
        self.publishCommand()
        
        



    def publishCommand(self):
        self.command_.header.frame_id = "map"
        self.command_.header.stamp = rospy.Time.now()

        self.command_.steer = np.clip(self.dl_, -max_steer, max_steer)
        self.command_.a = self.ai_
        self.command_.flag = self.flag_
        #
        print(self.command_.steer*180/3.14)
        #for debug
        #print("dl = ", self.dl_)

        self.command_pub_.publish(self.command_)
        #rospy.sleep(0.1)



if __name__ == '__main__':
    print("video control start!")
    rospy.init_node('video_control_node', anonymous=True)
    vcn = VideoControlNode()
    vcn.run()
    



