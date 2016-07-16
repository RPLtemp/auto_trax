//
// Created by marius on 16.04.16.
//

#include "auto_trax_controller/controller_processor.h"

namespace auto_trax {

void InitializeParameters(const ros::NodeHandle& nh, ParameterBag* parameter) {
  // Retrieve all parameters or set to default
  nh.param("subscribed_rostopic_setpoint",
           parameter->subscribed_rostopic_setpoint, kDefaultSetPointSubTopic);
  nh.param("queue_size_subscriber_setpoint",
           parameter->queue_size_subscriber_setpoint,
           kDefaultSetPointSubQueueSize);
  nh.param("subscribed_rostopic_plantstate",
           parameter->subscribed_rostopic_plantstate,
           kDefaultPlantStateSubTopic);
  nh.param("queue_size_subscriber_plantstate",
           parameter->queue_size_subscriber_plantstate,
           kDefaultPlantStateSubQueueSize);
  nh.param("pub_rostopic_control_effort",
           parameter->pub_rostopic_control_effort,
           kDefaultControlEffortPubTopic);
  nh.param("queue_size_pub_control_effort",
           parameter->queue_size_pub_control_effort,
           kDefaultControlEffortPubQueueSize);
  nh.param("service_rostopic_steering_angle",
           parameter->service_rostopic_steering_angle,
           kDefaultSteeringAngleServiceTopic);
  nh.param("setpoint", parameter->setpoint, kDefaultSetPoint);

  // Retrieve PID Parameters
  nh.param("Kp", parameter->pid_bag.Kp, kDefaultKp);
  nh.param("Ki", parameter->pid_bag.Ki, kDefaultKi);
  nh.param("Kd", parameter->pid_bag.Kd, kDefaultKd);
  nh.param("upper_limit", parameter->pid_bag.upper_limit, kDefaultUpperLimit);
  nh.param("lower_limit", parameter->pid_bag.lower_limit, kDefaultLowerLimit);
  nh.param("windup_limit", parameter->pid_bag.windup_limit,
           kDefaultWindupLimit);
  nh.param("cutoff_frequency", parameter->pid_bag.cutoff_frequency,
           kDefaultCutOffFrequency);
}

} // namespace auto_trax

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "auto_trax_controller_node");
  ros::NodeHandle nh;

  // Initialize parameter structure
  auto_trax::ParameterBag parameter;
  auto_trax::InitializeParameters(nh, &parameter);

  // Construct class detection_processor with NodeHandle and parameter structure
  auto_trax::ControllerProcessor processor(nh, parameter);

  // Spin
  ros::spin();
}
