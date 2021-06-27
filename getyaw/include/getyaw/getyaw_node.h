//
// Created by sheffieldwang on 2021/4/20.
//

#ifndef GETYAW_GETYAW_NODE_H
#define GETYAW_GETYAW_NODE_H

#include<cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

double roll, pitch, yaw;
double yaw_deg;
double origin_yaw;
double pub_yaw;
int count = 0;
geometry_msgs::PoseStamped pose_msg;

ros::Publisher pub;
ros::Publisher pub_pose;
ros::Subscriber sub;
std_msgs::Float64 yaw_data;

#endif //GETYAW_GETYAW_NODE_H
