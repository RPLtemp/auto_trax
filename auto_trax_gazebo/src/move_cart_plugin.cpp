/*
 * move_cart.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: pvechersky
 */

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/Float64.h>

namespace gazebo
{
  class MoveCart : public ModelPlugin
  {
    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
        // Store the pointer to the model
        model_ = _parent;

        // Get the pointer to the main link
        main_link_ = model_->GetLink("chassis");

        // Subscribe to receive the next desired position
        steering_sub_ = nh_.subscribe("control_effort", 10, &MoveCart::SteeringCallback, this);
      }

      void SteeringCallback(const std_msgs::Float64ConstPtr& msg) {
        // Get the orientation of the cart
        math::Quaternion orientation = model_->GetWorldPose().rot;

        // Linear velocity vector in body frame
        math::Vector3 linear_vel_B = math::Vector3(0.5, 0.0, 0.0);

        // Rotate the linear velocity into world frame
        math::Vector3 linear_vel_W = orientation.RotateVector(linear_vel_B);

        // Set the constant linear velocity
        model_->SetLinearVel(linear_vel_W);

        // Use the steering command as the angular velocity around the z axis to turn the cart
        model_->SetAngularVel(math::Vector3(0.0, 0.0, msg->data));
      }

    private:
      /// ROS node handle
      ros::NodeHandle nh_;

      /// Pointer to the model
      physics::ModelPtr model_;

      /// Pointer to the main link
      physics::LinkPtr main_link_;

      /// Subscriber to receive steering commands
      ros::Subscriber steering_sub_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveCart);
}
