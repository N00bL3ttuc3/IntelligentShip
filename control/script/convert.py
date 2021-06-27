#!/usr/bin/env python3
#import ROS
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
from control.msg import Command


class ConvertNode():
    def __init__(self):
        #ros
        # vertify the msg
        self.rec_flag = Bool()
        self.rec_flag.data=True
        #voice flag
        self.voice_flag = 1
        # vertify the path
        self.path_flag = Bool()
        self.path_flag.data=True


        # path from lidar
        self.lidar_path_sub_ = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.callbackFromLidarPath)
        # path from auto
        self.auto_path_sub_ = rospy.Subscriber("autopath", Path, self.callbackFromAutoPath)
        # path pub
        self.path_sub_ = rospy.Subscriber("path_vertify", Bool, self.callbackFromVertifyPath)

        # path_control
        self.path_vertify_pub_ = rospy.Publisher("finalpath",Path,queue_size=1)

        # stanley command sub
        self.stanley_command_sub_ = rospy.Subscriber("stanley_control_command", Command, self.callbackFromStanleyCommand)
        # rec command sub

        self.rec_command_sub_ = rospy.Subscriber("rec_control_command", Command, self.callbackFromRecCommand)
        # video command sub 
        self.rec_command_sub_ = rospy.Subscriber("video_control_command", Command, self.callbackFromVideoCommand)
        # command pub
        self.command_pub_ = rospy.Publisher("control_command",Command,queue_size=1)

        # vertify msg
        self.vertify_pub_ = rospy.Publisher("vertify",Bool,queue_size=1)



        #trajectory path
        self.tx_ = []
        self.ty_ = []
        self.goal_ = []
        self.pose_ = PoseStamped()
        #command
        self.command_ = Command()
        self.path_ =  Path()







        # voice sub
        self.voice_sub = rospy.Subscriber("Voice_flag",Int64,self.callbackFromVoice)

    def run(self):
        rospy.spin()
        
    def callbackFromVoice(self,msg):
        self.voice_flag = msg.data
        self.rec_flag.data=True
        self.vertify_pub_.publish(self.voice_flag)
        if(self.voice_flag == 0):
            print("AutoPath")
        if(self.voice_flag == 1):
            print("LidarPath")
            print("StanleyCommand")
        if(self.voice_flag == 3):
            print("RecCommand")
        if(self.voice_flag == 4):
            print("VideoCommand")

    def callbackFromVertifyPath(self,msg):
        self.path_flag.data = msg.data

    def callbackFromLidarPath(self,msg):
        print("ok")
        if(self.voice_flag == 1):
            self.path_.header = msg.header
            self.path_.poses = msg.poses
            self.path_vertify_pub_.publish(self.path_)
            print("pub!")


    def callbackFromAutoPath(self,msg):
        if(self.voice_flag == 0):
            self.path_.header = msg.header
            self.path_.poses = msg.poses 
            self.path_vertify_pub_.publish(self.path_)
            print("pub autopath!")

    def callbackFromStanleyCommand(self,msg):
        if(self.voice_flag == 1 or self.voice_flag == 0 ):
            self.command_.header.frame_id = "map"
            self.command_.header.stamp = rospy.Time.now()
            self.command_.a = msg.a 
            self.command_.steer = msg.steer 
            self.command_.flag = msg.flag 
            self.command_pub_.publish(self.command_)


    def callbackFromRecCommand(self,msg):
        if(self.voice_flag == 3):
            self.command_.header.frame_id = "map"
            self.command_.header.stamp = rospy.Time.now()
            self.command_.a = msg.a 
            self.command_.steer = msg.steer 
            self.command_.flag = msg.flag 
            self.command_pub_.publish(self.command_)
    
    def callbackFromVideoCommand(self,msg):
        if(self.voice_flag == 4):
            self.command_.header.frame_id = "map"
            self.command_.header.stamp = rospy.Time.now()
            self.command_.a = msg.a 
            self.command_.steer = msg.steer 
            self.command_.flag = msg.flag 
            self.command_pub_.publish(self.command_)


if __name__ == '__main__':
    print("convert start!")
    rospy.init_node('convert_node', anonymous=True)
    covn = ConvertNode()
    covn.run()

