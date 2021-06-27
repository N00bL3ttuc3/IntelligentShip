#!/usr/bin/env python3
"""
LQR Control

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

#Parameter of LQR
Kp = 1.0  # speed proportional gain
# LQR parameter
Q = np.eye(4)
R = np.eye(1)
R[0] = 1000

# parameters
dt = 0.1  # time tick[s]
L = 1.0  # width length of the boat [m]
goal_dis = 1 #[m]
target_speed = 3.6 / 3.6  # [m/s]
max_steer = np.radians(45.0)  # [rad] max steering angle

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

class LQRControlNode():
    def __init__(self):
        #ros
        #self.path_sub_ = rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.callbackFromPath)
        #self.path_sub_ = rospy.Subscriber("waypath", Path, self.callbackFromPath)
        self.state_ = State(x=-0.0, y=0.0, yaw=0.0, v=0.0)
        self.path_sub_ = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.callbackFromPath)

        self.pose_sub_ = rospy.Subscriber("boat_pose", PoseStamped, self.callbackFromPose)
        self.command_pub_ = rospy.Publisher("control_command",Command,queue_size=10)
        self.cubic_pub_ = rospy.Publisher("cubic_path",Path,queue_size=10)
        #trajectory path
        self.tx_ = []
        self.ty_ = []
        self.goal_ = []
        #state
        
        #error
        self.e_ = 0.0
        self.e_th_ = 0.0
        #command
        self.command_ = Command()
        self.dl_ = 0.0
        self.ai_ = 0.0
        self.flag_ = 0
        self.path_ =  Path()






    def run(self):
        rospy.spin()

    def callbackFromPath(self,path):
        print("received path")
    #    self.tx_.append(self.state_.x)
    #    self.ty_.append(self.state_.y)
     #   self.tx_.append(self.state_.x + 0.5 * math.cos(self.state_.yaw) )
     #   self.ty_.append(self.state_.y + 0.5 * math.sin(self.state_.yaw))

        for i in range(len(path.poses)):
        #     if( (path.poses[i].pose.position.x - self.state_.x)**2 + (path.poses[i].pose.position.y - self.state_.y)**2 > 0.35  ):
            self.tx_.append(path.poses[i].pose.position.x)
            self.ty_.append(path.poses[i].pose.position.y)
      #  print("ty = ",self.ty_,"\n")
      #  print("tx = ",self.tx_,"\n")
        self.goal_ = [self.tx_[-1],self.ty_[-1]]
        cx, cy, cyaw, ck, s = calc_spline_course(
            self.tx_, self.ty_, ds=0.1)

        sp = self.calc_speed_profile(cx, cy, cyaw, target_speed)

        self.dl_, target_ind, self.e_, self.e_th_ = self.lqr_steering_control(
            self.state_, cx, cy, cyaw, ck, self.e_, self.e_th_)
        print("e_th = ",self.e_th_)
        print("e_ = ", self.e_)
        self.ai_ = 0
        self.flag_ = 1
        
        #print(cy)
        for i in range(len(cx)):
            pose_tmp = PoseStamped()
            pose_tmp.pose.position.x = cx[i]
            pose_tmp.pose.position.y = cy[i]
            self.path_.header.frame_id = "slamware_map"
            self.path_.header.stamp = rospy.Time.now()
            self.path_.poses.append(pose_tmp)
        
        self.cubic_pub_.publish(self.path_)

        print("dl before = ",self.dl_)
        self.path_.poses = []
        

        # check goal
        dx = self.state_.x - self.goal_[0]
        dy = self.state_.y - self.goal_[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            self.ai_ = 0.0
            self.dl_ = 0.01
            self.flag_ = 0
        self.publishCommand()
        

        self.tx_ = []
        self.ty_ = []

    def callbackFromPose(self,pose):
        self.state_.x  = pose.pose.position.x
        self.state_.y  = pose.pose.position.y
        self.state_.v  = pose.pose.orientation.z
        self.state_.yaw = pose.pose.orientation.z 




    def publishCommand(self):
        self.command_.header.frame_id = "map"
        self.command_.header.stamp = rospy.Time.now()
        if(self.dl_ > max_steer):
            self.dl_ = max_steer
        if(self.dl_ < -max_steer):
            self.dl_ = -max_steer
        self.command_.steer = self.dl_
        self.command_.a = 0
        self.command_.flag = self.flag_
        print("dl after = ",self.command_.steer)

        self.command_pub_.publish(self.command_)
        #rospy.sleep(0.1)



    def calc_speed_profile(self,cx, cy, cyaw, target_speed):
        speed_profile = [target_speed] * len(cx)

        direction = 1.0

        # Set stop point
        for i in range(len(cx) - 1):
            dyaw = abs(cyaw[i + 1] - cyaw[i])
            switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

            if switch:
                direction *= -1

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

            if switch:
                speed_profile[i] = 0.0

        speed_profile[-1] = 0.0

        return speed_profile

    def calc_nearest_index(self,state, cx, cy, cyaw):
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind

    def PIDControl(self,target, current):
        a = Kp * (target - current)

        return a

    def lqr_steering_control(self,state, cx, cy, cyaw, ck, pe, pth_e):
        ind, e = self.calc_nearest_index(state, cx, cy, cyaw)

        k = ck[ind]
        v = state.v
        th_e = self.pi_2_pi(state.yaw - cyaw[ind])

        A = np.zeros((4, 4))
        A[0, 0] = 1.0
        A[0, 1] = dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = dt
        # print(A)

        B = np.zeros((4, 1))
        B[3, 0] = v / L

        K, _, _ = self.dlqr(A, B, Q, R)

        x = np.zeros((4, 1))

        x[0, 0] = e
        x[1, 0] = (e - pe) / dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / dt

        ff = math.atan2(L * k, 1)
        fb = self.pi_2_pi((-K @ x)[0, 0])

        delta = ff + fb

        return delta, ind, e, th_e

    def solve_DARE(self,A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        X = Q
        maxiter = 150
        eps = 0.01

        for i in range(maxiter):
            Xn = A.T @ X @ A - A.T @ X @ B @ \
                 la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
            if (abs(Xn - X)).max() < eps:
                break
            X = Xn

        return Xn

    def dlqr(self,A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_DARE(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eigVals, eigVecs = la.eig(A - B @ K)

        return K, X, eigVals
    def pi_2_pi(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

if __name__ == '__main__':
    print("lqr start!")
    rospy.init_node('pid_control_node', anonymous=True)
    lqrn = LQRControlNode()
    lqrn.run()
    



