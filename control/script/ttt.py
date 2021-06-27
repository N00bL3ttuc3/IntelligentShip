#!/usr/bin/env python3
"""
Stanley Control

Author: SheffieldWang
"""
#import basic
import math
import numpy as np
import scipy.linalg as la
import bisect


#import ROS
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from control.msg import Command
import sys

class StanleyControlNode():
    def __init__(self):
        #ros
        self.command_pub_ = rospy.Publisher("control_command",Command,queue_size=10)
        #command
        self.command_ = Command()
        self.count = 0






    def run(self):
        self.publishCommand()

   


    def publishCommand(self):
        self.command_.header.frame_id = "map"
        self.command_.header.stamp = rospy.Time.now()
        self.command_.steer = 0
        self.command_.a=0
        self.command_.flag = 0
        print("pub",self.count)
        self.command_pub_.publish(self.command_)
        rospy.sleep(0.1)

   

if __name__ == '__main__':

    print("stanley start!")
    rospy.init_node('stanley_control_node', anonymous=True)
    conn = StanleyControlNode()
    while(1):
        conn.run()
        conn.count = conn.count +1
