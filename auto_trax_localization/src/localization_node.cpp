#include "localization_node.hpp"
#include "particle_filter.hpp"
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
  mapSub       =nh_.subscribe("map", 1, &LocalizationNode::mapCB, this);

  particles_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  particle_laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("particle_laser_scan", 0);
  particles_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1);

  particleFilter_.setNParticles(100);
  particleFilter_.spawnParticles();
  ROS_INFO("Particles spawned!");
  int nToShow = 10;
  ROS_INFO("Displaying %i particles",nToShow);
  particleFilter_.show(nToShow);

  publishParticlesRViz();

  prev_encoder_FR_ = 0.0;
}

void LocalizationNode::depthScanCB(const sensor_msgs::LaserScanConstPtr &scan_msg)
{

  if (!laserScanParamsInitialized)
  {
    ROS_INFO("Initializing laser scan parameters");
    particleFilter_.initializeLaserScanParameters(scan_msg->angle_min, scan_msg->angle_max, scan_msg->angle_increment,
                                                  scan_msg->range_min, scan_msg->range_max);
    laserScanParamsInitialized = true;
    last_scan_msg_ptr = sensor_msgs::LaserScanPtr(new sensor_msgs::LaserScan);
  }

  *last_scan_msg_ptr = *scan_msg;

}

void LocalizationNode::encoderCB(const barc::EncoderConstPtr &encoder_msg)
{
  float delta_encoder_turns = encoder_msg->FR - prev_encoder_FR_;
  float delta_y = delta_encoder_turns * 0.08;
  float delta_x = 0.0;

  particleFilter_.propagate(delta_x, delta_y);

  prev_encoder_FR_ = encoder_msg->FR;

//  ROS_INFO("EncoderCallBack");
  publishParticlesRViz();
}


void LocalizationNode::mapCB(const nav_msgs::OccupancyGridConstPtr& map_msg){
  if (!mapParamsInitialized)
  {
    ROS_INFO("Initializing map parameters");
    particleFilter_.initializeMapParameters(map_msg->info.resolution, map_msg->info.width, map_msg->info.height,
                                            map_msg->info.origin.position.x, map_msg->info.origin.position.y,
                                            2*std::acos(map_msg->info.origin.orientation.w));
    std::vector<int> map;
    for (int i = 0; i < map_msg->data.size(); i++) map.push_back(map_msg->data[i]);
    particleFilter_.setMap(map);
    mapParamsInitialized = true;
  }

}

void LocalizationNode::publishParticlesRViz() {
  if (laserScanParamsInitialized && mapParamsInitialized) {
    std::vector<float> ranges;
    particleFilter_.extract_particle_local_scan(initial_pose_, ranges);
    sensor_msgs::LaserScan laserScan;
    laserScan = *last_scan_msg_ptr;
    laserScan.ranges = ranges;
    particle_laser_scan_pub_.publish(laserScan);
  }

//  visualization_msgs::Marker *marker = generateMarker(initial_pose_);
//  particles_pub_.publish(*marker);




  geometry_msgs::PoseArray particle_poses = particleFilter_.particlesToMarkers();
  particles_poses_pub_.publish(particle_poses);
}

void LocalizationNode::initializeParameters()
{
  initial_pose_ = boost::shared_ptr<WheelBot>(new WheelBot());

  float initial_x, initial_y, initial_theta;
  bool allParametersSet = true;
  nh_.param("initial_pose_x", initial_x, DefaultInitialX);
  nh_.param("initial_pose_y", initial_y, DefaultInitialY);
  nh_.param("initial_pose_theta", initial_theta, DefaultInitialTheta);
  nh_.param("particle_marker_length", particleVisualProperties.length, DefaultParticleVisualLength);
  nh_.param("particle_marker_width", particleVisualProperties.width, DefaultParticleVisualLength);
  nh_.param("particle_marker_height", particleVisualProperties.height, DefaultParticleVisualLength);

  ROS_INFO("Particle length %.2f",particleVisualProperties.length);

  initial_pose_->setPose(initial_x, initial_y, initial_theta);

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
