//
// Created by marius on 16.04.16.
//

#include "auto_trax_controller/controller_processor.h"

namespace auto_trax {

void InitializeParameters(ParameterBag* parameter) {
  // Retrieve all the necessary parameters
  ros::NodeHandle pnh("~");

  if (!pnh.getParam("subscribed_rostopic_setpoint",
                   parameter->subscribed_rostopic_setpoint)) {
    ROS_ERROR("Please specify subscribed setpoint topic for the controller");
    ros::shutdown();
  }
  pnh.param("queue_size_subscriber_setpoint",
           parameter->queue_size_subscriber_setpoint,
           kDefaultSetPointSubQueueSize);

  if (!pnh.getParam("subscribed_rostopic_plantstate",
                   parameter->subscribed_rostopic_plantstate)) {
    ROS_ERROR("Please specify subscribed plant state topic for the controller");
    ros::shutdown();
  }
  pnh.param("queue_size_subscriber_plantstate",
           parameter->queue_size_subscriber_plantstate,
           kDefaultPlantStateSubQueueSize);

  pnh.param("subscribed_rostopic_pathstate",
           parameter->subscribed_rostopic_pathstate,
           kDefaultPathStateTopic);
  pnh.param("queue_size_subscriber_pathstate",
           parameter->queue_size_subscriber_pathstate,
           kDefaultPathStateSubQueueSize);

  if (!pnh.getParam("pub_rostopic_control_effort",
                   parameter->pub_rostopic_control_effort)) {
    ROS_ERROR("Please specify published control effort topic for the controller");
    ros::shutdown();
  }
  pnh.param("queue_size_pub_control_effort",
           parameter->queue_size_pub_control_effort,
           kDefaultControlEffortPubQueueSize);

  if (!pnh.getParam("output_service_name", parameter->output_service_name)) {
    ROS_ERROR("Please specify output service name for the controller");
    ros::shutdown();
  }

  pnh.param("setpoint", parameter->setpoint, kDefaultSetPoint);

  // Retrieve PID Parameters
  pnh.param("Kp", parameter->pid_bag.Kp, kDefaultKp);
  pnh.param("Ki", parameter->pid_bag.Ki, kDefaultKi);
  pnh.param("Kd", parameter->pid_bag.Kd, kDefaultKd);
  pnh.param("Kp_angular_rate", parameter->angular_rate_pid_bag.Kp, kDefaultKp);
  pnh.param("Ki_angular_rate", parameter->angular_rate_pid_bag.Ki, kDefaultKi);
  pnh.param("Kd_angular_rate", parameter->angular_rate_pid_bag.Kd, kDefaultKd);
  pnh.param("upper_limit", parameter->pid_bag.upper_limit, kDefaultUpperLimit);
  pnh.param("lower_limit", parameter->pid_bag.lower_limit, kDefaultLowerLimit);
  pnh.param("windup_limit", parameter->pid_bag.windup_limit,
           kDefaultWindupLimit);
  pnh.param("cutoff_frequency", parameter->pid_bag.cutoff_frequency,
           kDefaultCutOffFrequency);
}

} // namespace auto_trax

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, argv[1]);
  ros::NodeHandle nh;

  // Initialize parameter structure
  auto_trax::ParameterBag parameter;
  auto_trax::InitializeParameters(&parameter);

  // Construct class detection_processor with NodeHandle and parameter structure
  auto_trax::ControllerProcessor processor(nh, parameter);

  // Spin
  ros::spin();
}
