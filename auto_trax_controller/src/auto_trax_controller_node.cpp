//
// Created by marius on 23.04.16.
//

#include "parameter/parameter_bag.h"
#include "controller_processor.h"

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "auto_trax_controller_node");
    ros::NodeHandle nh;

    // Initialize parameter structure
    ParameterBag parameter;

    // Retrieve all parameters
    nh.getParam("subscribed_rostopic_img", parameter.subscribed_rostopic_img);
    nh.getParam("queue_size_subscriber_img", parameter.queue_size_subscriber_img);
    nh.getParam("subscribed_rostopic_imu", parameter.subscribed_rostopic_imu);
    nh.getParam("queue_size_subscriber_imu", parameter.queue_size_subscriber_imu);

    // Construct class detection_processor with ros::NodeHandle and parameter structure
    ControllerProcessor controller (nh, parameter);

    // Create ROS subscriber for the image and imu
    ros::Subscriber sub_imu = nh.subscribe(parameter.subscribed_rostopic_imu,
                                          parameter.queue_size_subscriber_imu,
                                          &ControllerProcessor::CallbackImu,
                                          &controller);

    // Spin
    ros::spin ();
}