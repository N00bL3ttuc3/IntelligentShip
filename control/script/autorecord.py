#!/usr/bin/env python3
"""
Stanley Control

Author: SheffieldWang
"""
#import basic
import math
import numpy as np


#import ROS
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Int64,Bool
from geometry_msgs.msg import PoseStamped



class AutoRecord():
    def __init__(self):
        #Pose list
        self.px_list=[]
        self.py_list=[]
        self.old_x=0.0
        self.old_y=0.0

        #voice flag
        self.voice_flag = 1
        #ros
        self.pose_sub_ = rospy.Subscriber("boat_pose", PoseStamped, self.callbackFromPose)
        # voice sub
        self.voice_sub = rospy.Subscriber("Voice_flag",Int64,self.callbackFromVoice)
        self.path_pub_ = rospy.Publisher("autopath",Path,queue_size=1)
        #path
        self.path_ =  Path()


    def run(self):
        rospy.spin()

    def callbackFromPose(self,pose):
        if(self.voice_flag == 5):
            if((pose.pose.position.x - self.old_x)**2 + (pose.pose.position.y - self.old_y)**2 > 0.4**2):
                self.px_list.append(pose.pose.position.x)
                self.py_list.append(pose.pose.position.y)
                self.old_x = pose.pose.position.x
                self.old_y = pose.pose.position.y

    def callbackFromVoice(self,msg):
        self.voice_flag = msg.data
        if(self.voice_flag == 5):
            print("Record!")
        elif(self.voice_flag==0):
            for i in range(len(self.px_list)):
                pose_tmp = PoseStamped()
                pose_tmp.pose.position.x = self.px_list[i]
                pose_tmp.pose.position.y = self.py_list[i]
                self.path_.header.frame_id = "slamware_map"
                self.path_.header.stamp = rospy.Time.now()
                self.path_.poses.append(pose_tmp)

        self.path_pub_.publish(self.path_)
        


if __name__ == '__main__':
    print("auro record start!")
    rospy.init_node('auto_record_node', anonymous=True)
    atcn=AutoRecord()
    atcn.run()
    
