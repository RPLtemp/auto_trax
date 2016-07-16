#include <auto_trax_io/auto_trax_io_node.h>

namespace auto_trax {

AutoTraxIoNode::AutoTraxIoNode() {
  if (!InitializeParameters()) {
    ROS_ERROR("Geometric Parameters not imported");
    ros::shutdown();
  }

  m_upper_ = (params_.max_linear_velocity_m_s - params_.min_linear_velocity_m_s) /
          (params_.motor_max - params_.motor_upper_pwm_threshold);
  q_upper_ = params_.min_linear_velocity_m_s - m_upper_ * params_.motor_upper_pwm_threshold;

  m_lower_ = (-params_.min_linear_velocity_m_s + params_.max_linear_velocity_m_s) /
          (params_.motor_lower_pwm_threshold - params_.motor_min);
  q_lower_ = -params_.min_linear_velocity_m_s - m_lower_ * params_.motor_lower_pwm_threshold;

  ROS_DEBUG("m_lower: %f, q_lower: %f", m_lower_, q_lower_);
  ROS_DEBUG("m_upper: %f, q_upper: %f", m_upper_, q_upper_);

  steering_service_ = nh_.advertiseService(steering_service_name_,
                              &AutoTraxIoNode::SteeringServiceCallback, this);

  motor_service_ = nh_.advertiseService(motor_service_name_,
                              &AutoTraxIoNode::MotorServiceCallback, this);

  pwm_driver_ = new PCA9685(0x40);

  if (pwm_driver_->openPCA9685() < 0) {
    ROS_ERROR("COULD NOT OPEN PCA9685");
  }
  else {
    ROS_INFO("PCA9685 Device Address: 0x%02X\n",pwm_driver_->kI2CAddress) ;
    servo_range_ = params_.servo_max - params_.servo_min;
    pwm_driver_->setAllPWM(0,0) ;
    pwm_driver_->reset() ;
    pwm_driver_->setPWMFrequency(params_.pwm_frequency);
	sleep(1);
    pwm_driver_->setPWM(params_.speed_i2c_channel, 0, params_.zero_speed_pwm);
  }
}

AutoTraxIoNode::~AutoTraxIoNode() {
  pwm_driver_->setPWM(params_.speed_i2c_channel, 0, params_.zero_speed_pwm);
  pwm_driver_->closePCA9685();
}

inline double AutoTraxIoNode::AngleConversion(double angle_in_rads) {
  double positive_angle_in_rads = angle_in_rads + kAngleRangeInRadians / 2;
  return (servo_range_ * positive_angle_in_rads) /
          kAngleRangeInRadians + params_.servo_min;
}

inline double AutoTraxIoNode::SpeedConversion(double speed_in_m_s) {
  // Check if we are in the "dead zone" where the car cannot run
  if (std::abs(speed_in_m_s) < params_.min_linear_velocity_m_s)
    return params_.zero_speed_pwm;

  // Check if we are above saturation on both ends
  if (std::abs(speed_in_m_s) > params_.max_linear_velocity_m_s) {
    if (std::signbit(speed_in_m_s))
      return params_.motor_min;
    else
      return params_.motor_max;
  }

  // If speed is negative use lower equation
  if (std::signbit(speed_in_m_s)) {
    double pwm_result = (speed_in_m_s - q_lower_) / m_lower_;
    return static_cast<int>(pwm_result);
  }
  else { //else use upper eq
    double pwm_result = (speed_in_m_s - q_upper_) / m_upper_;
    return static_cast<int>(pwm_result);
  }

  // Just to be safe for testing purposes.
  return params_.zero_speed_pwm;
}

bool AutoTraxIoNode::SteeringServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                                             auto_trax_msgs::IOSetpoint::Response &res) {
  double angle_in_radians = req.setpoint;
  double pwm_angle = AngleConversion(angle_in_radians);

  ROS_DEBUG("Pwm angle: %f", pwm_angle);

  if (pwm_driver_->error >= 0 && parameter_initialized_) {
    pwm_driver_->setPWM(params_.angle_i2c_channel, 0, pwm_angle);
    res.success = true;
    return true;
  }

  res.success = false;
  return false;
}

bool AutoTraxIoNode::MotorServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                                          auto_trax_msgs::IOSetpoint::Response &res) {
  double speed_in_m_s = req.setpoint;
  double pwm_speed = SpeedConversion(speed_in_m_s);

  ROS_DEBUG("Pwm speed: %f", pwm_speed);

  if (pwm_driver_->error >= 0 && parameter_initialized_) {
    pwm_driver_->setPWM(params_.speed_i2c_channel, 0, pwm_speed);
    res.success = true;
    return true;
  }

  res.success = false;
  return false;
}

bool AutoTraxIoNode::InitializeParameters() {
  bool success = nh_.getParam("steering_service_name", steering_service_name_) &&
          nh_.getParam("motor_service_name", motor_service_name_) &&
          nh_.getParam("pwm_frequency", params_.pwm_frequency) &&
          nh_.getParam("angle_i2c_channel", params_.angle_i2c_channel) &&
          nh_.getParam("speed_i2c_channel", params_.speed_i2c_channel) &&
          nh_.getParam("servo_min", params_.servo_min) &&
          nh_.getParam("servo_max", params_.servo_max) &&
          nh_.getParam("motor_min", params_.motor_min) &&
          nh_.getParam("motor_max", params_.motor_max) &&
          nh_.getParam("min_linear_velocity_m_s", params_.min_linear_velocity_m_s) &&
          nh_.getParam("max_linear_velocity_m_s", params_.max_linear_velocity_m_s) &&
          nh_.getParam("motor_lower_pwm_threshold", params_.motor_lower_pwm_threshold) &&
          nh_.getParam("motor_upper_pwm_threshold", params_.motor_upper_pwm_threshold) &&
          nh_.getParam("zero_speed_pwm", params_.zero_speed_pwm);
  return success;
}

} // namespace auto_trax

int main(int argc, char **argv) {
  // Init ros
  ros::init(argc, argv, "auto trax io node");

  // Instantiate the io node
  auto_trax::AutoTraxIoNode io_node;

  // Spin
  ros::spin();
  return 0;
}
