//
// Created by marius on 12.06.16.
//

#ifndef AUTO_TRAX_CONTROLLER_PID_H
#define AUTO_TRAX_CONTROLLER_PID_H

#include <ros/ros.h>
#include <iostream>
#include "auto_trax_controller/parameter/pid_bag.h"

namespace auto_trax {

static float kDefaultKp = 3.0;
static float kDefaultKi = 0.05;
static float kDefaultKd = 0.5;

static float kDefaultUpperLimit  = 1.0;
static float kDefaultLowerLimit  = -1.0;
static float kDefaultWindupLimit = 10.0;

static float kDefaultCutOffFrequency = -1;

class PID {
 public:
  PID(PidBag pid_param);
  virtual ~PID();

  double GetControlEffort(const float& setpoint, const float& plant_state);

 private:
  PidBag pid_param_;
  ros::Time prev_time_;

  std::vector<double> error_(3,0);
  std::vector<double> filtered_error_(3,0);
  std::vector<double> error_deriv_(3,0);
  std::vector<double> filtered_error_deriv_(3,0);

  int loop_counter_;
};

} // namespace auto_trax

#endif //AUTO_TRAX_CONTROLLER_PID_H
