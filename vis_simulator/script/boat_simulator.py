#!/usr/bin/env python3
"""

Boat simulator

author SheffieldWang

"""
#import basic
import math
import numpy as np
import bisect

#import ROS
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from control.msg import Command

dt = 0.1  # time tick[s]
L = 0.5  # width length of the boat [m]
max_steer = np.deg2rad(90.0)  # maximum steering angle[rad]

class State:

    def __init__(self, x=0.0, y=0.0, yaw=np.radians(90), v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    #state.yaw = np.clip(state.yaw,-np.radians(45),np.radians(45))
    print("yaw = ",180*state.yaw/3.14)
    
    state.v = 0.3

    return state

class BoatSimulator():
    def __init__(self):
        self.state_ = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
        self.command_sub_ = rospy.Subscriber("control_command",Command,self.callbackFromCommand)
        self.boat_path_pub_ = rospy.Publisher("boat_path",Path,queue_size=10)
        self.pose_pub_ = rospy.Publisher("boat_pose",PoseStamped,queue_size=10)
        self.a_ = 0.0
        self.dl_ = 0.0
        #Path
        self.pose_ = PoseStamped()
        self.path_ =  Path()

    def run(self):
        print("yaw = ",180*self.state_.yaw/3.14)
        rospy.spin()

    def callbackFromCommand(self,command):
        self.a_ = command.a
        self.dl_ = command.steer
        self.state_ = update(self.state_,self.a_,self.dl_)
        #print("x = ",self.state_.x)
        #print("y = ",self.state_.y)
        self.pose_.pose.position.x = self.state_.x
        self.pose_.pose.position.y = self.state_.y
        self.pose_.pose.position.z = self.state_.v
        self.pose_.pose.orientation.z = self.state_.yaw
        
        pose_tmp = PoseStamped()
        pose_tmp.pose.position.x = self.state_.x
        pose_tmp.pose.position.y = self.state_.y
        self.path_.header.frame_id = "slamware_map"
        self.path_.header.stamp = rospy.Time.now()
        self.path_.poses.append(pose_tmp)
        
        self.pose_pub_.publish(self.pose_)
        self.boat_path_pub_.publish(self.path_)




if __name__ == '__main__':
    print("Boat Simulator Start!")
    rospy.init_node('boat_simulator', anonymous=True)
    bsn = BoatSimulator()
    bsn.run()
