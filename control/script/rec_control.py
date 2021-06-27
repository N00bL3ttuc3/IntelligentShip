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
from std_msgs.msg import Int64MultiArray
from control.msg import Command






class RecControlNode():
    def __init__(self):
        #ros
        self.path_sub_ = rospy.Subscriber("control_msg", Int64MultiArray, self.callbackFromControlmsg)
        self.command_pub_ = rospy.Publisher("rec_control_command",Command,queue_size=1)
        #command
        self.command_ = Command()
        self.dl_ = 0.0
        self.ai_ = 0.0
        self.flag_ = 0






    def run(self):
        rospy.spin()

    def callbackFromControlmsg(self,controlmsg):
        print(controlmsg.data)
        assert sum(controlmsg.data) == 1,"the sum of control_msg.data is not equal to 1"
        if controlmsg.data[2] == 1:
            self.flag_ = 1
        elif controlmsg.data[3] == 1:
            self.flag_ = 0
        if controlmsg.data[0] == 1:
            self.dl_ += math.pi*30/180
        elif controlmsg.data[1] == 1:
            self.dl_ -= math.pi*30/180

    
        self.publishCommand()
        



    def publishCommand(self):
        self.command_.header.frame_id = "map"
        self.command_.header.stamp = rospy.Time.now()

        if (self.dl_ > math.pi*30/180):
            self.dl_ = math.pi*30/180
        if self.dl_ < -math.pi*30/180:	    
            self.dl_ = -math.pi*30/180
        self.command_.steer = self.dl_
        self.command_.a = self.ai_
        self.command_.flag = self.flag_
        #for debug
        #print("dl = ", self.dl_)

        self.command_pub_.publish(self.command_)
        #rospy.sleep(0.1)



if __name__ == '__main__':
    print("rec control start!")
    rospy.init_node('rec_control_node', anonymous=True)
    recn = RecControlNode()
    recn.run()
    



