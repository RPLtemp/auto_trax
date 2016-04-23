//
// Created by marius on 23.04.16.
//

#include "controller_processor.h"

ControllerProcessor::ControllerProcessor(ros::NodeHandle nodehandle, ParameterBag params_bag) {
	nh_ = nodehandle;
	parameter_ = params_bag;
}

void ControllerProcessor::CallbackImu(const sensor_msgs::Imu::ConstPtr& imu_msg) {
	std::cout << "Callback imu!" << std::endl;
}