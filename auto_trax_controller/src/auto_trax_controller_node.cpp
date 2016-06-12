//
// Created by marius on 16.04.16.
//

#include "auto_trax_controller/parameter/parameter_bag.h"
#include "auto_trax_controller/controller_processor.h"

namespace auto_trax {

void InitializeParameters(const ros::NodeHandle& nh, ParameterBag* parameter) {
  // Retrieve all parameters or set to default
  nh.param("subscribed_rostopic_setpoint", parameter->subscribed_rostopic_setpoint, kDefaultSetPointSubTopic);
  nh.param("queue_size_subscriber_setpoint", parameter->queue_size_subscriber_setpoint, kDefaultSetPointSubQueueSize);
  nh.param("subscribed_rostopic_plantstate", parameter->subscribed_rostopic_plantstate, kDefaultPlantStateSubTopic);
  nh.param("queue_size_subscriber_plantstate", parameter->queue_size_subscriber_plantstate, kDefaultPlantStateSubQueueSize);
  nh.param("pub_rostopic_control_effort", parameter->pub_rostopic_control_effort, kDefaultControlEffortPubTopic);
  nh.param("queue_size_pub_control_effort", parameter->queue_size_pub_control_effort, kDefaultControlEffortPubQueueSize);
  nh.param("service_rostopic_steering_angle", parameter->service_rostopic_steering_angle, kDefaultSteeringAngleServiceTopic);
}

} // namespace auto_trax

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "auto_trax_controller_node");
  ros::NodeHandle nh;

  // Initialize parameter structure
  ParameterBag parameter;
  auto_trax::InitializeParameters(nh, &parameter);

  // Construct class detection_processor with ros::NodeHandle and parameter structure
  auto_trax::ControllerProcessor processor(nh, parameter);

  // Relative path to package
  std::string string = ros::package::getPath("auto_trax_controller");

  // Spin
  ros::spin ();
}
