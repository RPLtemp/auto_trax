//
// Created by marius on 12.06.16.
//

#include "auto_trax_controller/pid.h"

namespace auto_trax {

PID::PID(const PidBag& pid_param):
    pid_param_(pid_param) {
  error_.resize(3);
  filtered_error_.resize(3);
  error_deriv_.resize(3);
  filtered_error_deriv_.resize(3);
}

PID::~PID() {
}

double PID::GetControlEffort(const float& setpoint, const float& plant_state) {
  // All 3 gains should have the same sign
  if (!((pid_param_.Kp <= 0. && pid_param_.Ki <= 0. && pid_param_.Kd <= 0.)
      || (pid_param_.Kp >= 0. && pid_param_.Ki >= 0. && pid_param_.Kd >= 0.))) {
    ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  }

  std::cout << "Plant_state: " << plant_state << std::endl;

  error_.at(2) = error_.at(1);
  error_.at(1) = error_.at(0);
  error_.at(0) = plant_state - setpoint; // Current error goes to slot 0

  std::cout << "Error: " << error_.at(0) << std::endl;

  // Calculate delta_t
  ros::Duration delta_t;
  if (!prev_time_.isZero()) { // Not first time through the program
    delta_t = ros::Time::now() - prev_time_;
    prev_time_ = ros::Time::now();
    if (0 == delta_t.toSec()) {
      ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f",
                ros::Time::now().toSec());
      return 0.0;
    }
  } else {
    ROS_INFO("prev_time is 0, doing nothing");
    prev_time_ = ros::Time::now();
    return 0.0;
  }

  // integrate the error
  double error_integral;
  error_integral += error_.at(0) * delta_t.toSec();

  // Take derivative of error
  // First the raw, unfiltered data:
  error_deriv_.at(2) = error_deriv_.at(1);
  error_deriv_.at(1) = error_deriv_.at(0);
  error_deriv_.at(0) = (error_.at(0) - error_.at(1)) / delta_t.toSec();

  // calculate the control effort
  double proportional = pid_param_.Kp * error_.at(0);
  double integral = pid_param_.Ki * error_integral;
  double derivative = pid_param_.Kd * error_deriv_.at(0);
  double control_effort = proportional + integral + derivative;

  std::cout << "P: " << proportional << " | I: " << integral << " | D: "
      << derivative << std::endl;

  // Apply saturation limits
  if (control_effort > pid_param_.upper_limit) {
    control_effort = pid_param_.upper_limit;
    ROS_WARN("Control effort exceeded upper limit");
  } else if (control_effort < pid_param_.lower_limit) {
    control_effort = pid_param_.lower_limit;
    ROS_WARN("Control effort exceeded lower limit");
  }

  return control_effort;
}

} // namespace auto_trax
