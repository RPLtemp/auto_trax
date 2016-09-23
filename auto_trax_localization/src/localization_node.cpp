#include <include/localization_node.hpp>
#include <include/particle_filter.hpp>
#include <string>

LocalizationNode::LocalizationNode(ros::NodeHandle nh) : nh_(nh)

{

  ROS_WARN("Localization node%s started!", (
          nh_.getNamespace() == "/" ? "" : nh_.getNamespace().c_str()));
  depthScanSub = nh_.subscribe("merged_scan",1,&LocalizationNode::depthScanCB,
  this);
  encoderSub   = nh_.subscribe("encoder", 1, &LocalizationNode::encoderCB, this);

  particleFilter_.setNParticles(100);
  particleFilter_.spawnParticles();
  ROS_INFO("Particles spawned!");
  int nToShow = 10;
  ROS_INFO("Displaying %i particles",nToShow);
  particleFilter_.show(nToShow);

}

void LocalizationNode::depthScanCB(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
  ROS_INFO("DepthScanCallBack");
}

void LocalizationNode::encoderCB(const barc::EncoderConstPtr &encoder_msg)
{
  ROS_INFO("EncoderCallBack");
}
