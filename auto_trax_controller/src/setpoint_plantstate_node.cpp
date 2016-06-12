//
// Created by marius on 12.06.16.
//

#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_plantstate_node");
  ros::NodeHandle nh;

  while (ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Setpoint_node spinning waiting for time to become non-zero");
    sleep(1);
  }

  std_msgs::Float64 setpoint;
  std_msgs::Float64 state;

  setpoint.data = 0.5;
  state.data = 1.0;
  ros::Publisher setpoint_pub = nh.advertise<std_msgs::Float64>("/setpoint", 1);
  ros::Publisher state_pub = nh.advertise<std_msgs::Float64>("/plant_state", 1);


  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();

    // Publish setpoint
    setpoint_pub.publish(setpoint);

    // Publish plant state
    state.data += 1.0;
    state_pub.publish(state);

    loop_rate.sleep();
  }
}
