//
// Created by marius on 16.04.16.
//

#ifndef AUTO_TRAX_CONTROLLER_PROCESSOR_H
#define AUTO_TRAX_CONTROLLER_PROCESSOR_H

#include <auto_trax_io/IOSetpoint.h>
#include <std_msgs/Float64.h>

#include "auto_trax_controller/parameter/parameter_bag.h"
#include "auto_trax_controller/pid.h"

namespace auto_trax {
// Default values
static constexpr int kDefaultSetPointSubQueueSize       = 1;
static constexpr int kDefaultPlantStateSubQueueSize     = 1;
static constexpr int kDefaultControlEffortPubQueueSize  = 1;
static constexpr float kDefaultSetPoint = 0.0;

class ControllerProcessor {
 public:
  // Constructor with nodehandle and parameters
  ControllerProcessor(const ros::NodeHandle& nh, const ParameterBag& parameter);
  virtual ~ControllerProcessor();

  // Callback
  void CallbackSetPoint(const std_msgs::Float64& set_point_msg);
  void CallbackPlantState(const std_msgs::Float64& plant_state_msg);

 private:
  ros::NodeHandle nh_;
  ParameterBag parameter_;
  PID pid_;

  ros::Subscriber sub_set_point_;
  ros::Subscriber sub_plant_state_;
  ros::Publisher pub_control_effort_;
  ros::ServiceClient client_;

  float setpoint_;
};

} // namespace auto_trax

#endif //AUTO_TRAX_CONTROLLER_PROCESSOR_H
