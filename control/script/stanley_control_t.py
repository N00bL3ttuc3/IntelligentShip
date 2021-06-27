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


k =  0.5# control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 0.1  # [m] Wheel base of vehicle
target_speed = 3.6 / 3.6 
max_steer = np.radians(45.0)  # [rad] max steering angle
goal_dis = 0.8


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

class State(object):
    """
    Class representing the state of a boat.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class StanleyControlNode():
    def __init__(self):
        #ros
        self.state_ = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
        self.path_sub_ = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.callbackFromPath)
        self.pose_sub_ = rospy.Subscriber("boat_pose", PoseStamped, self.callbackFromPose)
        self.command_pub_ = rospy.Publisher("control_command",Command,queue_size=10)
        self.cubic_pub_ = rospy.Publisher("cubic_path",Path,queue_size=10)
        self.pose_pub_ = rospy.Publisher("target_pose",PoseStamped,queue_size=10)
        #trajectory path
        self.tx_ = []
        self.ty_ = []
        self.goal_ = []
        self.pose_ = PoseStamped()
        #state

        #index
        self.last_idx_ = 0
        self.target_idx_ = 0
        #command
        self.command_ = Command()
        self.dl_ = 0.0
        self.ai_ = 0.0
        self.flag_ = 0
        self.path_ =  Path()






    def run(self):
        rospy.spin()

    def callbackFromPath(self,path):
        for i in range(len(path.poses)):
            self.tx_.append(path.poses[i].pose.position.x)
            self.ty_.append(path.poses[i].pose.position.y)
        # print("tx",self.tx_)
        # print("ty",self.ty_)
        self.goal_ = [self.tx_[-1],self.ty_[-1]]
        cx, cy, cyaw, ck, s = calc_spline_course(
            self.tx_, self.ty_, ds=0.1)
        self.last_idx_ = len(cx) -1 
        # print("last_idex = ",self.last_idx_)
        
        self.target_idx_, _ = self.calc_target_index(self.state_, cx, cy)
        # print("cy ",cy)
        # print("state y",self.state_.y)
        # print("initial_idex = ",self.target_idx_)

        for i in range(len(cx)):
            pose_tmp = PoseStamped()
            pose_tmp.pose.position.x = cx[i]
            pose_tmp.pose.position.y = cy[i]
            self.path_.header.frame_id = "slamware_map"
            self.path_.header.stamp = rospy.Time.now()
            self.path_.poses.append(pose_tmp)

        self.cubic_pub_.publish(self.path_)
        self.path_.poses = []

        #   while self.last_idx_ > self.target_idx_:
        # print(self.last_idx_ - self.target_idx_)
        self.ai_ = self.pid_control(target_speed, self.state_.v)
        # print("input_idx ",self.target_idx_)
        #target_idx_tmp = np.clip(self.target_idx_ + 5, 0 ,self.last_idx_)
        self.dl_, self.target_idx_ = self.stanley_control(self.state_, cx, cy, cyaw, self.target_idx_)
        # print("current_idx ",self.target_idx_)
        # print("state y",self.state_.y)
        self.pose_.header.frame_id = "slamware_map"
        self.pose_.header.stamp = rospy.Time.now()
        self.pose_.pose.position.x = cx[self.target_idx_]
        self.pose_.pose.position.y = cy[self.target_idx_]
        self.pose_.pose.position.z =0
        self.pose_.pose.orientation.x = 0

        self.pose_pub_.publish(self.pose_)


    

        self.flag_ = 1
        

    
        

        # check goal
        dx = self.state_.x - self.goal_[0]
        dy = self.state_.y - self.goal_[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            # self.ai_ = 0.0
            # self.dl_ = 0.01
            # self.flag_ = 0
        self.publishCommand()
        

        self.tx_ = []
        self.ty_ = []

    def callbackFromPose(self,pose):
        self.state_.x  = pose.pose.position.x
        self.state_.y  = pose.pose.position.y
        self.state_.v  = pose.pose.position.z
        self.state_.yaw = pose.pose.orientation.z  #- np.radians(90)
        #print("yaw = ",180*self.state_.yaw/3.14)




    def publishCommand(self):
        self.command_.header.frame_id = "map"
        self.command_.header.stamp = rospy.Time.now()
        self.command_.steer = np.clip(self.dl_, -max_steer, max_steer)
        self.command_.a = self.ai_
        self.command_.flag = self.flag_
        print("dl = ",180 * (self.command_.steer)/3.14)
        self.command_pub_.publish(self.command_)
        rospy.sleep(0.1)

    def pid_control(self,target, current):
        return Kp * (target - current)

    def calc_target_index(self,state, cx, cy):

        # Calc front axle position
        fx = state.x + L * np.cos(state.yaw)
        fy = state.y + L * np.sin(state.yaw)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                        -np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

    def stanley_control(self,state, cx, cy, cyaw, last_target_idx):

        current_target_idx, error_front_axle = self.calc_target_index(state, cx, cy)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx
        # print("error_front_axle",error_front_axle)

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(cyaw[current_target_idx] - state.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, state.v)
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx

    def normalize_angle(self,angle):
        """
        Normalize an angle to [-pi, pi].
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

if __name__ == '__main__':
    print("stanley start!")
    rospy.init_node('stanley_control_node', anonymous=True)
    conn = StanleyControlNode()
    conn.run()
