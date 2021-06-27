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
from control.msg import Command



class AttackNode():
    def __init__(self):
        
        #command
        self.command_ = Command()
        self.dl_ = 0.0
        # 3 attack
        self.ai_ = 0
        # 0:no run 1:run
        self.flag_ = 1
        # attack_flag
        self.attack_flag=False

        #voice flag
        self.voice_flag = 0
        #ros
        self.brain_sub_ = rospy.Subscriber("BrainWave_flag",Bool,self.callbackFromBrain)
        # voice sub
        self.voice_sub = rospy.Subscriber("Voice_flag",Int64,self.callbackFromVoice)
        self.command_pub_ = rospy.Publisher("control_command",Command,queue_size=1)



    def run(self):
        rospy.spin()

    def callbackFromBrain(self,msg):
        if(self.voice_flag==2):
            self.attack_flag = msg.data
            if(self.attack_flag == True):
                self.ai_ = 3

            else:
                self.ai_=0
            #print(self.ai_)
            self.dl_=0
            self.flag_=0
            self.publishCommand()

        
    

    def callbackFromVoice(self,msg):
        self.voice_flag = msg.data
        if(self.voice_flag == 2):
            print("Attack")

    def publishCommand(self):
        self.command_.header.frame_id = "map"
        self.command_.header.stamp = rospy.Time.now()
        self.command_.steer = self.dl_
        self.command_.a = self.ai_
        self.command_.flag = self.flag_
        if(self.command_.a == 3):
            print("attack!!!")
        
        self.command_pub_.publish(self.command_)
        #rospy.sleep(0.1)

        


if __name__ == '__main__':
    print("attack node start!")
    rospy.init_node('attack_node', anonymous=True)
    atcn=AttackNode()
    atcn.run()
    
