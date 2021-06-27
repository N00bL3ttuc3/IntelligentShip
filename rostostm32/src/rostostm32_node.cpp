//
// Created by sheffieldwang on 2020/12/21.
//
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "rostostm32/rostostm32.h"
#include "control/Command.h"
 

//test send value

// double testSend1 = 5500;
// double testSend2 = 2200;
// unsigned char testSend3 = 0x08;

//test receive value

double testRece1 = 0.0;
double testRece2 = 0.0;
double testRece3 = 0.0;
unsigned char testRece4 = 0.0;

void commandCallback(const control::CommandPtr& msg)
{

    double yaw =  45+ 180*(-msg->steer)/M_PI;

    int v = msg->a;
    unsigned char flag = msg->flag;
    double realsteer = yaw-45;
    
    printf("Send data: v = %d, yaw = %f, flag = %d\n",v,realsteer,flag);
    std::cout<<"I send real steer is "<<realsteer<<std::endl;
    writeSpeed(v,yaw,flag);
    

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rostostm32_node");
    ros::NodeHandle nh;
    ros::Publisher flag_pub = nh.advertise<std_msgs::Bool>("stm32_flag",1000);
    ros::Subscriber command_sub=nh.subscribe("control_command",1000,commandCallback);

    
    serialInit();

    ros::spin();

    //ros::Rate loop_rate(10);

    // while(ros::ok())
    // {
    //     //ros::spinOnce();
    //     //writeSpeed(testSend1,testSend2,testSend3);
    //   //  readSpeed(testRece1,testRece2,testRece3,testRece4);
    //     ROS_INFO("%f,%f,%f,%d\n",testRece1,testRece2,testRece3,testRece4);

    //     loop_rate.sleep();
    // }
    return 0;
}
