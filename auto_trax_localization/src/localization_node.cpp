#include "localization_node.hpp"

static constexpr float DefaultInitialX = 20.0;
static constexpr float DefaultInitialY = 0.0;
static constexpr float DefaultInitialTheta = 0.0;
static constexpr float DefaultParticleVisualLength = 15;

LocalizationNode::LocalizationNode(ros::NodeHandle nh) : nh_(nh)
{
  initializeParameters();

  ROS_WARN("Localization node%s started!", (
          nh_.getNamespace() == "/" ? "" : nh_.getNamespace().c_str()));
  depthScanSub = nh_.subscribe("merged_scan",1,&LocalizationNode::depthScanCB, this);
  encoderSub   = nh_.subscribe("encoder", 1, &LocalizationNode::encoderCB, this);
  mapSub       = nh_.subscribe("map", 1, &LocalizationNode::mapCB, this);
  steering_service_ = nh_.advertiseService(steering_service_name_,
                                           &LocalizationNode::SteeringServiceCallback, this);

  motor_service_ = nh_.advertiseService(motor_service_name_,
                                        &LocalizationNode::MotorServiceCallback, this);

  particle_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  particle_laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("particle_laser_scan", 0);
  particles_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1);

  particleFilter_.setNParticles(100);
  particleFilter_.spawnParticles(*initial_pose_);
  ROS_INFO("Particles spawned!");
  int nToShow = 10;
  ROS_INFO("Displaying %i particles",nToShow);
  particleFilter_.show(nToShow);

  prev_encoder_FR_ = 0.0;
}

LocalizationNode::~LocalizationNode()
{
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


  particleFilter_.GetParticleWeights(scan_msg);
  boost::shared_ptr<WheelBot> pose_estimate = boost::shared_ptr<WheelBot> (new WheelBot());
  pose_estimate = particleFilter_.Resample();


//  std::cout << "X: " << pose_estimate->getX() << " Y: " << pose_estimate->getY() <<std::endl;
  publishPoseTF(pose_estimate);

  publishParticleRViz(pose_estimate);
}

void LocalizationNode::encoderCB(const barc::EncoderConstPtr &encoder_msg)
{
  float delta_encoder_turns = encoder_msg->FR - prev_encoder_FR_;
  float delta_y = delta_encoder_turns * 0.08;
  float delta_x = 0.0;

  particleFilter_.propagate(delta_x, delta_y);

  prev_encoder_FR_ = encoder_msg->FR;

  publishParticlesRViz();
}

bool LocalizationNode::SteeringServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                                               auto_trax_msgs::IOSetpoint::Response &res) {
  angle_in_radians_ = req.setpoint;
  return true;
}

bool LocalizationNode::MotorServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                                            auto_trax_msgs::IOSetpoint::Response &res) {
  speed_in_m_s_ = req.setpoint;
  return true;
}

void LocalizationNode::ForwardKinematics(Eigen::Matrix<float, 3, 1>& robot_velocity){
  float beta = static_cast<float>(angle_in_radians_);
  float l = 0.3; //m
  float r = 0.0512; //m

  Eigen::Matrix<float, 4, 3> A;
  A <<
           0,          1,             0,
  cosf(beta), sinf(beta), -l*sinf(beta),
           -1,          0,            0,
  -sinf(beta), cosf(beta), l*sinf(beta);

  Eigen::Matrix<float, 4, 2> B;
  B <<
  r, 0,
  0, r,
  0, 0,
  0, 0;

  // A*xi_dot_robot = B*phi_dot
  // Forward kinematics: xi_dot_robot = (A.transpose * A).inverse * A.transose * B * phi_dot
  Eigen::Matrix<float, 2, 1> phi_dot;
  phi_dot << speed_in_m_s_, speed_in_m_s_; // TODO: Change velocity of steerable wheel entry 2
  //  robot_velocity = (A.transpose() * A).inverse() * A.transpose() * B * phi_dot;

  Eigen::Matrix<float, 3, 4> A_trans;
  A_trans << A.transpose();
  Eigen::Matrix<float, 3, 3> temp1;
  temp1 << (A_trans*A);
  Eigen::Matrix<float, 3, 3> temp1_inv;
  temp1_inv = (temp1.inverse());
  Eigen::Matrix<float, 3, 4> temp2;
  temp2 << (temp1_inv*A_trans);
  Eigen::Matrix<float, 3, 2> temp3;
  temp3 << (temp2*B);
  robot_velocity << (temp3*phi_dot);
}

void LocalizationNode::GetGlobalPosition(geometry_msgs::PointPtr robot_position,
                                         const geometry_msgs::Point& start_position,
                                         const Eigen::Matrix<float, 3, 1>& robot_velocity,
                                         const double& delta_time){
  robot_position->x = start_position.x + robot_velocity(1,1) * delta_time;
  robot_position->y = start_position.y + robot_velocity(2,1) * delta_time;
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

void LocalizationNode::publishParticlesRViz()
{
  geometry_msgs::PoseArray particle_poses = particleFilter_.particlesToMarkers();
  particles_poses_pub_.publish(particle_poses);
}

void LocalizationNode::publishParticleRViz(boost::shared_ptr<WheelBot> particle)
{
  if (laserScanParamsInitialized && mapParamsInitialized) {
    /*std::vector<float> ranges;
    particleFilter_.extract_particle_local_scan(particle, ranges);
    sensor_msgs::LaserScan laserScan;
    laserScan = *last_scan_msg_ptr;
    laserScan.angle_min = particleFilter_.getLaserScanParams().angle_min;
    laserScan.angle_max = particleFilter_.getLaserScanParams().angle_max;
    laserScan.angle_increment = particleFilter_.getLaserScanParams().angle_increment;
    laserScan.header.frame_id = "robot";
    laserScan.ranges = ranges;
    particle_laser_scan_pub_.publish(laserScan);*/
  }

  visualization_msgs::Marker *marker = generateMarker(particle);
  particle_pub_.publish(*marker);
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
  nh_.getParam("steering_service_name", steering_service_name_);
  nh_.getParam("motor_service_name", motor_service_name_);

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

void LocalizationNode::publishPoseTF(boost::shared_ptr<WheelBot> particle)
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(particle->getX(), particle->getY(), 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, particle->getTheta());
  transform.setRotation(q);
  pose_br_.sendTransform(tf::StampedTransform(transform, last_scan_msg_ptr->header.stamp, "map", "robot"));
}
