//
// Created by sheffieldwang on 2021/4/20.
//

#include "getyaw/getyaw_node.h"



// void callbackfromIMU(const sensor_msgs::Imu::ConstPtr msg)
// {
//     tf::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w);
//     tf::Matrix3x3 m(q);
//     m.getRPY(roll, pitch, yaw);
//     yaw_data.data = yaw;
//     yaw_deg = 180*(M_PI + yaw)/M_PI; 
//     ROS_INFO("The yaw is %lf",yaw_deg);
//     pub.publish(yaw_data);
// }


void callbackfromLidar(const nav_msgs::Odometry::ConstPtr &msg){

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
     tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    if(count == 0)
    {
      origin_yaw = yaw;
      count++;
    }
    else
    {
      pub_yaw = yaw - origin_yaw;
    }
    yaw_data.data = pub_yaw;
    yaw_deg = 180*(pub_yaw)/M_PI; 
 // ROS_INFO("The yaw is %lf",180*yaw/M_PI);
  pose_msg.header = msg->header;
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;
  pose_msg.pose.orientation.z = yaw;

    pub_pose.publish(pose_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"getyaw");
    ros::NodeHandle n;
  //  sub = n.subscribe("/imu",1000,callbackfromIMU);
    sub = n.subscribe("/slamware_ros_sdk_server_node/odom",1000,callbackfromLidar);
    pub = n.advertise<std_msgs::Float64>("imu/yaw",1000);
    pub_pose = n.advertise<geometry_msgs::PoseStamped>("boat_pose",1000);
    ros::spin();

    return 0;

}
