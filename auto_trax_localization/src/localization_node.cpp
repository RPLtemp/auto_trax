#include <include/localization_node.hpp>
#include <include/particle_filter.hpp>
#include <string>

static constexpr float DefaultInitialX = 20.0;
static constexpr float DefaultInitialY = 0.0;
static constexpr float DefaultInitialTheta = 0.0;
static constexpr float DefaultParticleVisualLength = 15;

LocalizationNode::LocalizationNode(ros::NodeHandle nh) : nh_(nh)

{
  initializeParameters();

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
//  ROS_INFO("DepthScanCallBack");
}

void LocalizationNode::encoderCB(const barc::EncoderConstPtr &encoder_msg)
{
//  ROS_INFO("EncoderCallBack");
  publishParticleRViz();
}

void LocalizationNode::publishParticleRViz()
{
  visualization_msgs::Marker* marker = generateMarker(initial_pose_);
  particles_pub_.publish( *marker );

}

void LocalizationNode::initializeParameters()
{
  initial_pose_ = boost::shared_ptr<WheelBot>(new WheelBot());

  float initial_x, initial_y, initial_theta;
  bool allParametersSet = true;
  nh_.param("initial_pose_x", initial_x,DefaultInitialX);
  nh_.param("initial_pose_y", initial_y,DefaultInitialY);
  nh_.param("initial_pose_theta", initial_theta,DefaultInitialTheta);
  nh_.param("particle_marker_length", particleVisualProperties.length,DefaultParticleVisualLength);
  nh_.param("particle_marker_width", particleVisualProperties.width,DefaultParticleVisualLength);
  nh_.param("particle_marker_height", particleVisualProperties.height,DefaultParticleVisualLength);

  ROS_INFO("Particle length %.2f",particleVisualProperties.length);

  initial_pose_->setPose(initial_x,initial_y,initial_theta);

  if (!allParametersSet) ROS_WARN("Some crucial parameters aren't set!");
  if (allParametersSet) ROS_WARN("All parameters are set!");

}

visualization_msgs::Marker* LocalizationNode::generateMarker(boost::shared_ptr<WheelBot> particle)
{
  visualization_msgs::Marker* marker = new visualization_msgs::Marker();
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = "particles";
  marker->id = 0;
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->pose.position.x = particle->getX();
  marker->pose.position.y = particle->getY();
  marker->pose.position.z = 0.0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = sin(particle->getTheta()/2);
  marker->pose.orientation.w = cos(particle->getTheta()/2);
  marker->scale.x = particleVisualProperties.length;
  marker->scale.y = particleVisualProperties.width;
  marker->scale.z = particleVisualProperties.height;
  marker->color.a = 1.0; // Don't forget to set the alpha!
  marker->color.r = 0.0;
  marker->color.g = 1.0;
  marker->color.b = 0.0;
  marker->lifetime = ros::Duration(1000);
  return marker;
}
