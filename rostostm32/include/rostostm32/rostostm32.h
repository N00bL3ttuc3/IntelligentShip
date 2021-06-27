#ifndef ROS_TO_STM32_H
#define ROS_TO_STM32_H

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>

extern void serialInit();
extern void writeSpeed(double RobotV, double YawRate,unsigned char ctrlFlag);
extern bool readSpeed(double &vx,double &vth,double &th,unsigned char &ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
