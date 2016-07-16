#ifndef AUTO_TRAX_IO_NODE_H
#define AUTO_TRAX_IO_NODE_H

#include <auto_trax_msgs/IOSetpoint.h>
#include <ros/ros.h>

#include "auto_trax_io/JHPWMPCA9685.h"

namespace auto_trax {
// Constants
static constexpr double kAngleRangeInRadians = 2.26893;

struct AutoTraxIoParameters{
  int servo_max;
  int servo_min;
  int pwm_frequency;
  int angle_i2c_channel;
  int speed_i2c_channel;
  double min_linear_velocity_m_s;
  double max_linear_velocity_m_s;
  int zero_speed_pwm;
  int motor_max;
  int motor_min;
  int motor_lower_pwm_threshold;
  int motor_upper_pwm_threshold;
};

class AutoTraxIoNode {
 public:
  AutoTraxIoNode();
  ~AutoTraxIoNode();

  bool SteeringServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                               auto_trax_msgs::IOSetpoint::Response &res);

  bool MotorServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                            auto_trax_msgs::IOSetpoint::Response &res);

  bool InitializeParameters();

 private:
  ros::NodeHandle nh_;

  ros::ServiceServer steering_service_;
  ros::ServiceServer motor_service_;

  std::string steering_service_name_;
  std::string motor_service_name_;

  PCA9685 *pwm_driver_;

  double m_upper_;
  double m_lower_;
  double q_upper_;
  double q_lower_;

  int servo_range_;
  bool parameter_initialized_;

  AutoTraxIoParameters params_;

  inline double AngleConversion(double angle_in_radians);
  inline double SpeedConversion(double speed_in_m_s);
};

} // namespace auto_trax

#endif // AUTO_TRAX_IO_NODE_H
