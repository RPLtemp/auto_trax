//
// Created by marius on 16.04.16.
//

#include "auto_trax_controller/controller_processor.h"

namespace auto_trax {

ControllerProcessor::ControllerProcessor(const ros::NodeHandle& nodehandle,
                                         const ParameterBag& params_bag):
    nh_(nodehandle),
    params_(params_bag),
    pid_(params_.pid_bag),
    setpoint_(params_.setpoint),
    path_valid_(false) {
  ROS_DEBUG("Auto_trax PID Processor started!");

  // Create Subscriber for the set point and plant state
  sub_set_point_ = nh_.subscribe(params_.subscribed_rostopic_setpoint,
                                 params_.queue_size_subscriber_setpoint,
                                 &ControllerProcessor::CallbackSetPoint, this);
  sub_plant_state_ = nh_.subscribe(params_.subscribed_rostopic_plantstate,
                                   params_.queue_size_subscriber_plantstate,
                                   &ControllerProcessor::CallbackPlantState,
                                   this);
  sub_path_state_ = nh_.subscribe(params_.subscribed_rostopic_pathstate,
                                  params_.queue_size_subscriber_pathstate,
                                  &ControllerProcessor::CallbackPathState,
                                  this);

  // Create Publisher for control effort
  pub_control_effort_ = nh_.advertise<std_msgs::Float64>(
      params_.pub_rostopic_control_effort,
      params_.queue_size_pub_control_effort);

  // Service for applying the controller output
  ros::service::waitForService(params_.output_service_name);
  client_ = nh_.serviceClient<auto_trax_msgs::IOSetpoint>(
      params_.output_service_name);
}

ControllerProcessor::~ControllerProcessor(){
}

void ControllerProcessor::CallbackSetPoint(const std_msgs::Float64ConstPtr &setpoint_msg) {
  ROS_DEBUG("Set Point received!");
  setpoint_ = setpoint_msg->data;
}

void ControllerProcessor::CallbackPlantState(const std_msgs::Float64ConstPtr &state_msg) {
  ROS_DEBUG("Plant State received!");

  double control_effort;

  if (path_valid_) {
    float plant_state = state_msg->data;
    control_effort = pid_.GetControlEffort(setpoint_, plant_state);
  }
  else {
    ROS_WARN("Path not valid, publishing control effort of zero");
    control_effort = 0.0;
  }

  // Publish the stabilizing control effort
  std_msgs::Float64 control_msg;
  control_msg.data = control_effort;
  pub_control_effort_.publish(control_msg);

  ROS_DEBUG("Control_effort: %f", control_effort);

  float io_setpoint = static_cast<float>(control_effort);
  auto_trax_msgs::IOSetpoint srv;
  srv.request.setpoint = io_setpoint;

  if (client_.call(srv)) {
    ROS_INFO("IO Setpoint: %d | %f \n", srv.response.success,
             srv.request.setpoint);
  } else {
    ROS_INFO("Failed to call service!!");
  }
}

void ControllerProcessor::CallbackPathState(const std_msgs::BoolConstPtr& path_state_msg) {
  path_valid_ = path_state_msg->data;
}

} // namespace auto_trax
