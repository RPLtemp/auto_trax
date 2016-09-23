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

  particles_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  particleFilter_.setNParticles(100);
  particleFilter_.spawnParticles();
  ROS_INFO("Particles spawned!");
  int nToShow = 10;
  ROS_INFO("Displaying %i particles",nToShow);
  particleFilter_.show(nToShow);

  publishParticleRViz();
}

void LocalizationNode::depthScanCB(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
  ROS_INFO("DepthScanCallBack");
}

void LocalizationNode::encoderCB(const barc::EncoderConstPtr &encoder_msg)
{
  ROS_INFO("EncoderCallBack");
  publishParticleRViz();
}

void LocalizationNode::publishParticleRViz()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "particles";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  boost::shared_ptr<WheelBot> particle = particleFilter_.getParticle(0);
  marker.pose.position.x = particle->getX();
  marker.pose.position.y = particle->getY();
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = sin(particle->getTheta()/2);
  marker.pose.orientation.w = cos(particle->getTheta()/2);
  marker.scale.x = 0.5;
  marker.scale.y = 0.2;
  marker.scale.z = 0.35;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(1000);
//only if using a MESH_RESOURCE marker type:
  particles_pub_.publish( marker );

}
