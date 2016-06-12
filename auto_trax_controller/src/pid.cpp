//
// Created by marius on 12.06.16.
//

#include "auto_trax_controller/pid.h"

namespace auto_trax {

PID::PID(PidBag pid_param):
    pid_param_(pid_param) {
}

PID::~PID() {
}

double PID::GetControlEffort(const float& setpoint, const float& plant_state) {
  if (!((pid_param_.Kp <= 0. && pid_param_.Ki <= 0. && pid_param_.Kd <= 0.)
      || (pid_param_.Kp >= 0. && pid_param_.Ki >= 0. && pid_param_.Kd >= 0.))) // All 3 gains should have the same sign
  {
    ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  }

  std::cout << "Plant_state: " << plant_state << std::endl;

  error_.at(2) = error_.at(1);
  error_.at(1) = error_.at(0);
  error_.at(0) = plant_state - setpoint; // Current error goes to slot 0

  std::cout << "Error: " << error_.at(0) << std::endl;

  // Calculate delta_t
  ros::Duration delta_t;
  if (!prev_time_.isZero()) // Not first time through the program
  {
    delta_t = ros::Time::now() - prev_time_;
    prev_time_ = ros::Time::now();
    if (0 == delta_t.toSec()) {
      ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
      return 0.0;
    }
  }
  else {
    ROS_INFO("prev_time is 0, doing nothing");
    prev_time_ = ros::Time::now();
    return 0.0;
  }

  // integrate the error
  double error_integral;
  error_integral += error_.at(0) * delta_t.toSec();

  // Apply windup limit to limit the size of the integral term
  if (error_integral > fabsf(pid_param_.windup_limit))
    error_integral = fabsf(pid_param_.windup_limit);

  if (error_integral < -fabsf(pid_param_.windup_limit))
    error_integral = -fabsf(pid_param_.windup_limit);

  // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
  double c;
  if (pid_param_.cutoff_frequency != -1) {
    // Check if tan(_) is really small, could cause c = NaN
    double tan_filt = tan((pid_param_.cutoff_frequency * 6.2832) * delta_t.toSec() / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt <= 0.) && (tan_filt > -0.01))
      tan_filt = -0.01;
    if ((tan_filt >= 0.) && (tan_filt < 0.01))
      tan_filt = 0.01;

    c = 1 / tan_filt;
  }

  filtered_error_.at(2) = filtered_error_.at(1);
  filtered_error_.at(1) = filtered_error_.at(0);
  filtered_error_.at(0) = (1 / (1 + c * c + 1.414 * c))
      * (error_.at(2) + 2 * error_.at(1) + error_.at(0) - (c * c - 1.414 * c + 1) * filtered_error_.at(2)
          - (-2 * c * c + 2) * filtered_error_.at(1));

  // Take derivative of error
  // First the raw, unfiltered data:
  error_deriv_.at(2) = error_deriv_.at(1);
  error_deriv_.at(1) = error_deriv_.at(0);
  error_deriv_.at(0) = (error_.at(0) - error_.at(1)) / delta_t.toSec();

  filtered_error_deriv_.at(2) = filtered_error_deriv_.at(1);
  filtered_error_deriv_.at(1) = filtered_error_deriv_.at(0);


  if (loop_counter_ > 2) // Let some data accumulate
    filtered_error_deriv_.at(0) = (1 / (1 + c * c + 1.414 * c))
        * (error_deriv_.at(2) + 2 * error_deriv_.at(1) + error_deriv_.at(0)
            - (c * c - 1.414 * c + 1) * filtered_error_deriv_.at(2) - (-2 * c * c + 2) * filtered_error_deriv_.at(1));
  else
    loop_counter_++;

  // calculate the control effort
  double proportional = pid_param_.Kp * filtered_error_.at(0);
  double integral = pid_param_.Ki * error_integral;
  double derivative = pid_param_.Kd * filtered_error_deriv_.at(0);
  double control_effort = proportional + integral + derivative;

  std::cout << "P: " << proportional << " | I: " << integral << " | D: " << derivative << std::endl;

  // Apply saturation limits
  if (control_effort > pid_param_.upper_limit) {
    control_effort = pid_param_.upper_limit;
    ROS_WARN("Control effort exceeded upper limit");
  }
  else if (control_effort < pid_param_.lower_limit) {
    control_effort = pid_param_.lower_limit;
    ROS_WARN("Control effort exceeded lower limit");
  }

  return control_effort;
}

} // namespace auto_trax
