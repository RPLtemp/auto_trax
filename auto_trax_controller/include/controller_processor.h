//
// Created by marius on 23.04.16.
//

#ifndef AUTO_TRAX_CONTROLLER_CONTROLLER_PROCESSOR_H
#define AUTO_TRAX_CONTROLLER_CONTROLLER_PROCESSOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <vector>

#include "parameter/parameter_bag.h"

class ControllerProcessor
{
public:
  // Constructor with nodehandle and parameters
  ControllerProcessor (ros::NodeHandle nh, ParameterBag parameter);

  // Callback
  void CallbackImu (const sensor_msgs::Imu::ConstPtr& a_imu_msg);

private:
  ros::NodeHandle nh_;
  ParameterBag parameter_;
};


#endif //AUTO_TRAX_CONTROLLER_CONTROLLER_PROCESSOR_H
