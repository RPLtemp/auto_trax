//
// Created by marius on 16.04.16.
//

#ifndef AUTO_TRAX_PID_PARAMETER_BAG_H
#define AUTO_TRAX_PID_PARAMETER_BAG_H

#include <string>

#include "auto_trax_controller/parameter/pid_bag.h"

namespace auto_trax {

struct ParameterBag {
  // Include other bags
  PidBag pid_bag;

  // Parameter specific to parameter bag
  std::string node_name;

  std::string subscribed_rostopic_setpoint;
  int queue_size_subscriber_setpoint;

  std::string subscribed_rostopic_plantstate;
  int queue_size_subscriber_plantstate;

  std::string pub_rostopic_control_effort;
  int queue_size_pub_control_effort;

  std::string service_rostopic_steering_angle;

  float setpoint;
};

} // namespace auto_trax

#endif //AUTO_TRAX_PID_PARAMETER_BAG_H
