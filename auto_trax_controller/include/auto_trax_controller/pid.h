//
// Created by marius on 12.06.16.
//

#ifndef AUTO_TRAX_CONTROLLER_PID_H
#define AUTO_TRAX_CONTROLLER_PID_H

#include <iostream>
#include <ros/ros.h>

#include "auto_trax_controller/parameter/pid_bag.h"

namespace auto_trax {

static constexpr float kDefaultKp = 3.0;
static constexpr float kDefaultKi = 0.05;
static constexpr float kDefaultKd = 0.5;

static constexpr float kDefaultUpperLimit  = 1.0;
static constexpr float kDefaultLowerLimit  = -1.0;
static constexpr float kDefaultWindupLimit = 10.0;

static constexpr float kDefaultCutOffFrequency = -1;

class PID {
 public:
  PID(const PidBag& pid_param);
  virtual ~PID();

  double GetControlEffort(const float& setpoint, const float& plant_state);

 private:
  PidBag pid_param_;
  ros::Time prev_time_;

  std::vector<double> error_;
  std::vector<double> filtered_error_;
  std::vector<double> error_deriv_;
  std::vector<double> filtered_error_deriv_;

  int loop_counter_;
};

} // namespace auto_trax

#endif //AUTO_TRAX_CONTROLLER_PID_H
